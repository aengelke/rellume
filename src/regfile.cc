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
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm-c/Core.h>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>
#include <type_traits>



/**
 * \defgroup LLRegFile Register File
 * \brief Representation of a register file
 *
 * @{
 **/

namespace rellume {

class DeferredValueBase {
public:
    using Generator = llvm::Value*(*)(X86Reg, Facet, llvm::BasicBlock*, void*);

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

    llvm::Value* get(X86Reg reg, Facet facet, llvm::BasicBlock* bb) {
        if (generator) {
            values[0] = generator(reg, facet, bb, values);
            assert(values[0] != nullptr && "generator returned nullptr");
            generator = nullptr;
        }
        return static_cast<llvm::Value*>(values[0]);
    }
    explicit operator bool() const {
        return generator || values[0];
    }
};

template<typename T>
class DeferredValue : public DeferredValueBase {
public:
    using Generator = llvm::Value*(*)(X86Reg, Facet, llvm::BasicBlock*, T*);
    DeferredValue(Generator generator, T data)
            : DeferredValueBase(reinterpret_cast<DeferredValueBase::Generator>(generator)) {
        static_assert(std::is_trivially_copyable<T>::value, "invalid defer arg type");
        static_assert(sizeof(T) <= sizeof(values), "defer arg type too big");
        static_assert(alignof(T) <= alignof(void*), "defer arg type misaligned");
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
    bool has(Facet v) const {
        return table.b[static_cast<int>(v)] > 0;
    }
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
const typename ValueMap<R, E...>::template LookupTable<Facet::Value, sizeof...(E), Facet::MAX> ValueMap<R, E...>::table({E...});

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
    impl() : insert_block(nullptr) {}

    llvm::BasicBlock* GetInsertBlock() {
        return insert_block;
    }
    void SetInsertBlock(llvm::BasicBlock* new_block) {
        insert_block = new_block;
    }

    void Clear();
    void InitWithPHIs(std::vector<PhiDesc>*);

    llvm::Value* GetReg(X86Reg reg, Facet facet);
    void SetReg(X86Reg reg, Facet facet, llvm::Value*, bool clear_facets);

private:
    llvm::BasicBlock* insert_block;
    ValueMapGp<DeferredValueBase> regs_gp[16];
    ValueMapSse<DeferredValueBase> regs_sse[16];
    DeferredValueBase reg_ip;
    ValueMapFlags<DeferredValueBase> flags;

    DeferredValueBase* AccessRegFacet(X86Reg reg, Facet facet);
    llvm::Value* GetRegFacet(X86Reg reg, Facet facet);
};

void RegFile::impl::Clear() {
    for (unsigned i = 0; i < 16; i++)
        regs_gp[i].clear();
    for (unsigned i = 0; i < 16; i++)
        regs_sse[i].clear();
    flags.clear();
    reg_ip = nullptr;
}

void RegFile::impl::InitWithPHIs(std::vector<PhiDesc>* desc_vec) {
    auto fn = [desc_vec] (Facet) {
        using DeferData = std::vector<PhiDesc>*;
        return DeferredValue<DeferData>([](X86Reg reg, Facet facet,
                                           llvm::BasicBlock* bb,
                                           DeferData* defer_data) {
            llvm::IRBuilder<> irb(bb, bb->begin());
            auto phi = irb.CreatePHI(facet.Type(irb.getContext()), 4);
            (*defer_data)->push_back(std::make_tuple(reg, facet, phi));
            return llvm::cast<llvm::Value>(phi);
        }, desc_vec);
    };

    for (unsigned i = 0; i < 16; i++)
        regs_gp[i].setAll(fn);
    for (unsigned i = 0; i < 16; i++)
        regs_sse[i].setAll(fn);
    flags.setAll(fn);
    reg_ip = fn(Facet::I64);
}

DeferredValueBase* RegFile::impl::AccessRegFacet(X86Reg reg, Facet facet) {
    unsigned idx = reg.Index();
    switch (reg.Kind()) {
    case X86Reg::RegKind::GP:
        if (regs_gp[idx].has(facet))
            return &regs_gp[idx][facet];
        return nullptr;
    case X86Reg::RegKind::IP:
        if (facet == Facet::I64)
            return &reg_ip;
        return nullptr;
    case X86Reg::RegKind::EFLAGS:
        if (flags.has(facet))
            return &flags[facet];
        return nullptr;
    case X86Reg::RegKind::VEC:
        if (regs_sse[idx].has(facet))
            return &regs_sse[idx][facet];
        return nullptr;
    default:
        return nullptr;
    }
}

llvm::Value* RegFile::impl::GetRegFacet(X86Reg reg, Facet facet) {
    DeferredValueBase* def_val = AccessRegFacet(reg, facet);
    if (def_val)
        return def_val->get(reg, facet, insert_block);
    return nullptr;
}

llvm::Value*
RegFile::impl::GetReg(X86Reg reg, Facet facet)
{
    DeferredValueBase* facet_entry = AccessRegFacet(reg, facet);
    // If we store the selected facet in our register file and the facet is
    // valid, return it immediately.
    if (llvm::Value* res = GetRegFacet(reg, facet))
        return res;

    llvm::LLVMContext& ctx = insert_block->getContext();

    llvm::IRBuilder<> builder(ctx);
    llvm::Instruction* terminator = insert_block->getTerminator();
    if (terminator != NULL)
        builder.SetInsertPoint(terminator);
    else
        builder.SetInsertPoint(insert_block);

    llvm::Type* facetType = facet.Type(ctx);

    if (reg.Kind() == X86Reg::RegKind::GP)
    {
        llvm::Value* res = nullptr;
        llvm::Value* native = GetRegFacet(reg, Facet::I64);
        assert(native && "native gp-reg facet is null");
        switch (facet)
        {
        case Facet::I64:
        case Facet::I32:
        case Facet::I16:
        case Facet::I8:
            res = builder.CreateTrunc(native, facetType);
            break;
        case Facet::I8H:
            res = builder.CreateLShr(native, builder.getInt64(8));
            res = builder.CreateTrunc(res, builder.getInt8Ty());
            break;
        case Facet::PTR:
            // For pointer facets, it actually only matters that the value *is*
            // a pointer -- we don't actually care about the type.
            res = builder.CreateIntToPtr(native, builder.getInt8PtrTy());
            break;
        default:
            assert(false && "invalid facet for gp-reg");
            break;
        }

        if (facet_entry != nullptr)
            *facet_entry = res;
        return res;
    }
    else if (reg.Kind() == X86Reg::RegKind::IP)
    {
        llvm::Value* native = GetRegFacet(reg, Facet::I64);
        if (facet == Facet::I64)
            return native;
        else if (facet == Facet::PTR)
            return builder.CreateIntToPtr(native, builder.getInt8PtrTy());
        else
            assert(false && "invalid facet for ip-reg");
    }
    else if (reg.Kind() == X86Reg::RegKind::VEC)
    {
        llvm::Value* res = nullptr;
        llvm::Value* native = GetRegFacet(reg, Facet::IVEC);
        assert(native && "native sse-reg facet is null");
        switch (facet)
        {
        case Facet::I128:
            res = builder.CreateTrunc(native, facetType);
            break;
        case Facet::I8:
            res = GetReg(reg, Facet::V16I8);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::I16:
            res = GetReg(reg, Facet::V8I16);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::I32:
            res = GetReg(reg, Facet::V4I32);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::I64:
            res = GetReg(reg, Facet::V2I64);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::F32:
            res = GetReg(reg, Facet::V4F32);
            res = builder.CreateExtractElement(res, int{0});
            break;
        case Facet::F64:
            res = GetReg(reg, Facet::V2F64);
            res = builder.CreateExtractElement(res, int{0});
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
            int targetBits = facetType->getPrimitiveSizeInBits();
            llvm::Type* elementType = facetType->getVectorElementType();
            int elementBits = elementType->getPrimitiveSizeInBits();

            // Prefer 128-bit SSE facet over full vector register.
            if (targetBits <= 128)
                if (llvm::Value* value_128 = GetRegFacet(reg, Facet::I128))
                    native = value_128;
            int nativeBits = native->getType()->getPrimitiveSizeInBits();

            int targetCnt = facetType->getVectorNumElements();
            int nativeCnt = nativeBits / elementBits;

            // Cast native facet to appropriate type
            llvm::Type* nativeVecType = llvm::VectorType::get(elementType, nativeCnt);
            res = builder.CreateBitCast(native, nativeVecType);

            // If a shorter vector is required, use shufflevector.
            if (nativeCnt > targetCnt)
            {
                llvm::SmallVector<uint32_t, 16> mask;
                for (int i = 0; i < targetCnt; i++)
                    mask.push_back(i);
                llvm::Value* undef = llvm::UndefValue::get(nativeVecType);
                res = builder.CreateShuffleVector(res, undef, mask);
            }
            break;
        }
        default:
            assert(false && "invalid facet for sse-reg");
            break;
        }

        if (facet_entry != nullptr)
            *facet_entry = res;
        return res;
    }

    assert(0);

    return nullptr;
}

void
RegFile::impl::SetReg(X86Reg reg, Facet facet, llvm::Value* value, bool clearOthers)
{
    if (facet == Facet::PTR)
        assert(value->getType()->isPointerTy());
    else
        assert(value->getType() == facet.Type(insert_block->getContext()));

    if (clearOthers) {
        if (reg.Kind() == X86Reg::RegKind::GP) {
            assert(facet == Facet::I64);
            regs_gp[reg.Index()].clear();
        } else if (reg.Kind() == X86Reg::RegKind::VEC) {
            assert(facet == Facet::IVEC);
            regs_sse[reg.Index()].clear();
        }
    }

    DeferredValueBase* facet_entry = AccessRegFacet(reg, facet);
    assert(facet_entry && "attempt to store invalid facet");
    *facet_entry = value;
}

RegFile::RegFile() : pimpl{std::make_unique<impl>()} {}
RegFile::~RegFile() {}

llvm::BasicBlock* RegFile::GetInsertBlock() {
    return pimpl->GetInsertBlock();
}
void RegFile::SetInsertBlock(llvm::BasicBlock* new_block) {
    pimpl->SetInsertBlock(new_block);
}
void RegFile::Clear() {
    pimpl->Clear();
}
void RegFile::InitWithPHIs(std::vector<PhiDesc>* desc_vec) {
    pimpl->InitWithPHIs(desc_vec);
}
llvm::Value* RegFile::GetReg(X86Reg reg, Facet facet) {
    return pimpl->GetReg(reg, facet);
}
void RegFile::SetReg(X86Reg reg, Facet facet, llvm::Value* value, bool clear) {
    pimpl->SetReg(reg, facet, value, clear);
}

} // namespace

/**
 * @}
 **/
