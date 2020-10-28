/**
 * This file is part of Rellume.
 *
 * (c) 2016-2019, Alexis Engelke <alexis.engelke@googlemail.com>
 *
 * Rellume is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL)
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * Rellume is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Rellume.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \file
 **/

#include "regfile.h"

#include "facet.h"

#include <llvm-c/Core.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Intrinsics.h>

#include <cassert>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>
#include <type_traits>

namespace rellume {

unsigned RegisterSetBitIdx(ArchReg reg, Facet facet) {
    switch (reg.Kind()) {
    case ArchReg::RegKind::GP:
        return 0 + reg.Index();
    case ArchReg::RegKind::IP:
        return 16;
    case ArchReg::RegKind::EFLAGS:
        switch (facet) {
        // clang-format off
        case Facet::OF: return 17 + 0; break;
        case Facet::SF: return 17 + 1; break;
        case Facet::ZF: return 17 + 2; break;
        case Facet::AF: return 17 + 3; break;
        case Facet::PF: return 17 + 4; break;
        case Facet::CF: return 17 + 5; break;
        case Facet::DF: return 17 + 6; break;
        default: assert(false && "invalid facet for EFLAGS register");
        }
        // clang-format on
    case ArchReg::RegKind::VEC:
        return 24 + reg.Index();
    default:
        assert(false && "invalid register kind");
    }
    return 0xffffffff;
}

class DeferredValueBase {
public:
    using Generator = llvm::Value* (*)(ArchReg, Facet, llvm::BasicBlock*, void*);

protected:
    // If value is nullptr, then the generator (unless that is null as well)
    // is used to get the actual value.
    void* values[3];
    Generator generator;

public:
    DeferredValueBase() {}
    DeferredValueBase(llvm::Value* value) : values{value}, generator(nullptr) {}
    explicit DeferredValueBase(Generator generator) : generator(generator) {
        assert(generator && "deferred value with null generator");
    }

    DeferredValueBase(DeferredValueBase&& rhs) = default;
    DeferredValueBase& operator=(DeferredValueBase&& rhs) = default;

    DeferredValueBase(DeferredValueBase const&) = delete;
    DeferredValueBase& operator=(const DeferredValueBase&) = delete;

    llvm::Value* get(ArchReg reg, Facet facet, llvm::BasicBlock* bb) {
        if (generator) {
            values[0] = generator(reg, facet, bb, values);
            assert(values[0] != nullptr && "generator returned nullptr");
            generator = nullptr;
        }
        return static_cast<llvm::Value*>(values[0]);
    }
    explicit operator bool() const { return generator || values[0]; }
};

template<typename T>
class DeferredValue : public DeferredValueBase {
public:
    using Generator = llvm::Value* (*)(ArchReg, Facet, llvm::BasicBlock*, T*);
    DeferredValue(Generator generator, T data)
        : DeferredValueBase(
              reinterpret_cast<DeferredValueBase::Generator>(generator)) {
        static_assert(std::is_trivially_copyable<T>::value,
                      "invalid defer arg type");
        static_assert(sizeof(T) <= sizeof(values), "defer arg type too big");
        static_assert(alignof(T) <= alignof(void*),
                      "defer arg type misaligned");
        *reinterpret_cast<T*>(values) = data;
    }
};

template<typename R, Facet::Value... E>
class ValueMap {
    template<typename T, int N, int M>
    struct LookupTable {
        constexpr LookupTable(std::initializer_list<T> il) : f(), b() {
            int i = 0;
            for (auto elem : il) {
                f[i] = elem;
                b[static_cast<int>(elem)] = 1 + i++;
            }
        }
        T f[N];
        unsigned b[M];
    };

    static const LookupTable<Facet::Value, sizeof...(E), Facet::MAX> table;
    R values[sizeof...(E)];

public:
    bool has(Facet v) const { return table.b[static_cast<int>(v)] > 0; }
    R& operator[](Facet v) {
        assert(has(v));
        return values[table.b[static_cast<int>(v)] - 1];
    }
    template<typename F>
    void setAll(const F& init_func) {
        for (unsigned i = 0; i < sizeof...(E); i++)
            values[i] = init_func(table.f[i]);
    }
    void clear() {
        setAll([](Facet f) { return nullptr; });
    }
};
template<typename R, Facet::Value... E>
const typename ValueMap<R, E...>::template LookupTable<Facet::Value,
                                                       sizeof...(E), Facet::MAX>
    ValueMap<R, E...>::table({E...});

template<typename R>
using ValueMapGp = ValueMap<R, Facet::I64, Facet::I32, Facet::I16, Facet::I8, Facet::I8H, Facet::PTR>;

template<typename R>
using ValueMapSse = ValueMap<R, Facet::I128,
#if LL_VECTOR_REGISTER_SIZE >= 256
    Facet::I256,
#endif
    Facet::I8, Facet::V16I8,
    Facet::I16, Facet::V8I16,
    Facet::I32, Facet::V4I32,
    Facet::I64, Facet::V2I64,
    Facet::F32, Facet::V4F32,
    Facet::F64, Facet::V2F64
>;

template<typename R>
using ValueMapFlags = ValueMap<R, Facet::ZF, Facet::SF, Facet::PF, Facet::CF, Facet::OF, Facet::AF, Facet::DF>;


class RegFile::impl {
public:
    impl() : insert_block(nullptr), regs_gp{}, regs_sse{}, reg_ip(), flags(),
             dirty_regs(), cleaned_regs() {}

    llvm::BasicBlock* GetInsertBlock() { return insert_block; }
    void SetInsertBlock(llvm::BasicBlock* n) { insert_block = n; }

    void Clear();
    void InitWithPHIs(std::vector<PhiDesc>*, bool all_facets);

    llvm::Value* GetReg(ArchReg reg, Facet facet);
    void SetReg(ArchReg reg, Facet facet, llvm::Value*, bool clear_facets);

    RegisterSet& DirtyRegs() { return dirty_regs; }
    RegisterSet& CleanedRegs() { return cleaned_regs; }

private:
    llvm::BasicBlock* insert_block;
    ValueMapGp<DeferredValueBase> regs_gp[16];
    ValueMapSse<DeferredValueBase> regs_sse[16];
    DeferredValueBase reg_ip;
    ValueMapFlags<DeferredValueBase> flags;

    RegisterSet dirty_regs;
    RegisterSet cleaned_regs;

    DeferredValueBase* AccessRegFacet(ArchReg reg, Facet facet);
    llvm::Value* GetRegFacet(ArchReg reg, Facet facet);
};

void RegFile::impl::Clear() {
    for (unsigned i = 0; i < 16; i++)
        regs_gp[i].clear();
    for (unsigned i = 0; i < 16; i++)
        regs_sse[i].clear();
    flags.clear();
    reg_ip = nullptr;
}

void RegFile::impl::InitWithPHIs(std::vector<PhiDesc>* desc_vec,
                                 bool all_facets) {
    auto fn = [desc_vec](Facet) {
        using DeferData = std::vector<PhiDesc>*;
        return DeferredValue<DeferData>(
            [](ArchReg reg, Facet facet, llvm::BasicBlock* bb,
               DeferData* defer_data) {
                llvm::IRBuilder<> irb(bb, bb->begin());
                auto phi = irb.CreatePHI(facet.Type(irb.getContext()), 4);
                (*defer_data)->push_back(std::make_tuple(reg, facet, phi));
                return llvm::cast<llvm::Value>(phi);
            },
            desc_vec);
    };

    if (all_facets) {
        for (auto& reg : regs_gp)
            reg.setAll(fn);
        for (auto& reg : regs_sse)
            reg.setAll(fn);
    } else {
        for (auto& reg : regs_gp)
            reg[Facet::I64] = fn(Facet::I64);
        for (auto& reg : regs_sse)
            reg[Facet::IVEC] = fn(Facet::IVEC);
    }

    flags.setAll(fn);
    reg_ip = fn(Facet::I64);
}

DeferredValueBase* RegFile::impl::AccessRegFacet(ArchReg reg, Facet facet) {
    unsigned idx = reg.Index();
    switch (reg.Kind()) {
    case ArchReg::RegKind::GP:
        if (regs_gp[idx].has(facet))
            return &regs_gp[idx][facet];
        return nullptr;
    case ArchReg::RegKind::IP:
        if (facet == Facet::I64)
            return &reg_ip;
        return nullptr;
    case ArchReg::RegKind::EFLAGS:
        if (flags.has(facet))
            return &flags[facet];
        return nullptr;
    case ArchReg::RegKind::VEC:
        if (regs_sse[idx].has(facet))
            return &regs_sse[idx][facet];
        return nullptr;
    default:
        return nullptr;
    }
}

llvm::Value* RegFile::impl::GetRegFacet(ArchReg reg, Facet facet) {
    DeferredValueBase* def_val = AccessRegFacet(reg, facet);
    if (def_val)
        return def_val->get(reg, facet, insert_block);
    return nullptr;
}

llvm::Value* RegFile::impl::GetReg(ArchReg reg, Facet facet) {
    // If we store the selected facet in our register file and the facet is
    // valid, return it immediately.
    if (llvm::Value* res = GetRegFacet(reg, facet))
        return res;

    llvm::IRBuilder<> irb(insert_block);
    if (llvm::Instruction* terminator = insert_block->getTerminator())
        irb.SetInsertPoint(terminator);

    llvm::Type* facetType = facet.Type(insert_block->getContext());

    if (reg.Kind() == ArchReg::RegKind::GP) {
        llvm::Value* res = nullptr;
        llvm::Value* native = GetRegFacet(reg, Facet::I64);
        assert(native && "native gp-reg facet is null");
        switch (facet) {
        case Facet::I64:
        case Facet::I32:
        case Facet::I16:
        case Facet::I8:
            res = irb.CreateTrunc(native, facetType);
            break;
        case Facet::I8H:
            res = irb.CreateLShr(native, irb.getInt64(8));
            res = irb.CreateTrunc(res, irb.getInt8Ty());
            break;
        case Facet::PTR:
            // For pointer facets, it actually only matters that the value *is*
            // a pointer -- we don't actually care about the type.
            res = irb.CreateIntToPtr(native, irb.getInt8PtrTy());
            break;
        default:
            assert(false && "invalid facet for gp-reg");
            break;
        }

        if (DeferredValueBase* facet_entry = AccessRegFacet(reg, facet))
            *facet_entry = res;
        return res;
    } else if (reg.Kind() == ArchReg::RegKind::IP) {
        llvm::Value* native = GetRegFacet(reg, Facet::I64);
        if (facet == Facet::I64)
            return native;
        else if (facet == Facet::PTR)
            return irb.CreateIntToPtr(native, irb.getInt8PtrTy());
        else
            assert(false && "invalid facet for ip-reg");
    } else if (reg.Kind() == ArchReg::RegKind::VEC) {
        llvm::Value* res = nullptr;
        llvm::Value* native = GetRegFacet(reg, Facet::IVEC);
        assert(native && "native sse-reg facet is null");
        switch (facet) {
        case Facet::I128:
            res = irb.CreateTrunc(native, facetType);
            break;
        case Facet::I8:
            res = irb.CreateExtractElement(GetReg(reg, Facet::V16I8), int{0});
            break;
        case Facet::I16:
            res = irb.CreateExtractElement(GetReg(reg, Facet::V8I16), int{0});
            break;
        case Facet::I32:
            res = irb.CreateExtractElement(GetReg(reg, Facet::V4I32), int{0});
            break;
        case Facet::I64:
            res = irb.CreateExtractElement(GetReg(reg, Facet::V2I64), int{0});
            break;
        case Facet::F32:
            res = irb.CreateExtractElement(GetReg(reg, Facet::V4F32), int{0});
            break;
        case Facet::F64:
            res = irb.CreateExtractElement(GetReg(reg, Facet::V2F64), int{0});
            break;
        case Facet::V1I8:
        case Facet::V2I8:
        case Facet::V4I8:
        case Facet::V8I8:
        case Facet::V16I8:
        case Facet::V1I16:
        case Facet::V2I16:
        case Facet::V4I16:
        case Facet::V8I16:
        case Facet::V1I32:
        case Facet::V2I32:
        case Facet::V4I32:
        case Facet::V1I64:
        case Facet::V2I64:
        case Facet::V1F32:
        case Facet::V2F32:
        case Facet::V4F32:
        case Facet::V1F64:
        case Facet::V2F64: {
            llvm::Type* elem_ty = facetType->getVectorElementType();
            int targetCnt = facetType->getVectorNumElements();

            // Prefer 128-bit SSE facet over full vector register.
            if (facetType->getPrimitiveSizeInBits() <= 128)
                if (llvm::Value* value_128 = GetRegFacet(reg, Facet::I128))
                    native = value_128;

            int elementBits = elem_ty->getPrimitiveSizeInBits();
            int nativeBits = native->getType()->getPrimitiveSizeInBits();
            int nativeCnt = nativeBits / elementBits;

            // Cast native facet to appropriate type
            auto native_vec_ty = llvm::VectorType::get(elem_ty, nativeCnt);
            res = irb.CreateBitCast(native, native_vec_ty);

            // If a shorter vector is required, use shufflevector.
            if (nativeCnt > targetCnt) {
                llvm::SmallVector<uint32_t, 16> mask;
                for (int i = 0; i < targetCnt; i++)
                    mask.push_back(i);
                llvm::Value* undef = llvm::UndefValue::get(native_vec_ty);
                res = irb.CreateShuffleVector(res, undef, mask);
            }
            break;
        }
        default:
            assert(false && "invalid facet for sse-reg");
            break;
        }

        if (DeferredValueBase* facet_entry = AccessRegFacet(reg, facet))
            *facet_entry = res;
        return res;
    } else {
        assert(false && "GetReg with invalid register kind");
    }

    return nullptr;
}

void RegFile::impl::SetReg(ArchReg reg, Facet facet, llvm::Value* value,
                           bool clearOthers) {
    if (facet == Facet::PTR)
        assert(value->getType()->isPointerTy());
    else
        assert(value->getType() == facet.Type(insert_block->getContext()));

    if (clearOthers) {
        if (reg.Kind() == ArchReg::RegKind::GP) {
            assert(facet == Facet::I64);
            regs_gp[reg.Index()].clear();
        } else if (reg.Kind() == ArchReg::RegKind::VEC) {
            assert(facet == Facet::IVEC);
            regs_sse[reg.Index()].clear();
        }
    }

    if (llvm::isa<llvm::PHINode>(value)) {
        llvm::IRBuilder<> irb(insert_block);
        value = irb.CreateUnaryIntrinsic(llvm::Intrinsic::ssa_copy, value);
    }

    DeferredValueBase* facet_entry = AccessRegFacet(reg, facet);
    if (!clearOthers && !facet_entry)
        return; // It's an optional facet we don't keep in the regfile.
    assert(facet_entry && "attempt to store invalid facet");
    *facet_entry = value;

    dirty_regs[RegisterSetBitIdx(reg, facet)] = true;
}

RegFile::RegFile() : pimpl{std::make_unique<impl>()} {}
RegFile::~RegFile() {}

llvm::BasicBlock* RegFile::GetInsertBlock() { return pimpl->GetInsertBlock(); }
void RegFile::SetInsertBlock(llvm::BasicBlock* n) { pimpl->SetInsertBlock(n); }
void RegFile::Clear() { pimpl->Clear(); }
void RegFile::InitWithPHIs(std::vector<PhiDesc>* desc_vec, bool all_facets) {
    pimpl->InitWithPHIs(desc_vec, all_facets);
}
llvm::Value* RegFile::GetReg(ArchReg r, Facet f) { return pimpl->GetReg(r, f); }
void RegFile::SetReg(ArchReg reg, Facet facet, llvm::Value* value, bool clear) {
    pimpl->SetReg(reg, facet, value, clear);
}
RegisterSet& RegFile::DirtyRegs() { return pimpl->DirtyRegs(); }
RegisterSet& RegFile::CleanedRegs() { return pimpl->CleanedRegs(); }

} // namespace rellume
