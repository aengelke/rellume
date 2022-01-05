
#include <llvm/MC/MCAssembler.h>
#include <llvm/MC/MCAsmBackend.h>
#include <llvm/MC/MCAsmInfo.h>
#include <llvm/MC/MCCodeEmitter.h>
#include <llvm/MC/MCContext.h>
#include <llvm/MC/MCObjectFileInfo.h>
#include <llvm/MC/MCObjectWriter.h>
#include <llvm/MC/MCParser/MCAsmParser.h>
#include <llvm/MC/MCParser/MCTargetAsmParser.h>
#include <llvm/MC/MCSection.h>
#include <llvm/MC/MCStreamer.h>
#include <llvm/MC/MCTargetOptions.h>
#include <llvm/MC/MCValue.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Support/SourceMgr.h>
#include <llvm/Support/TargetRegistry.h>
#include <llvm/Support/TargetSelect.h>

#include <iostream>
#include <iomanip>


class PlainObjectWriter : public llvm::MCObjectWriter {
    llvm::raw_pwrite_stream& stream;

public:
    PlainObjectWriter(llvm::raw_pwrite_stream& stream) : stream(stream) {}

    void executePostLayoutBinding(llvm::MCAssembler& Asm, const llvm::MCAsmLayout& Layout) override {}
    void recordRelocation(llvm::MCAssembler &Asm, const llvm::MCAsmLayout &Layout,
                          const llvm::MCFragment *Fragment,
                          const llvm::MCFixup &Fixup, llvm::MCValue Target,
                          uint64_t &FixedValue) override {
        assert(false && "relocations not supported");
    }
    uint64_t writeObject(llvm::MCAssembler &Asm, const llvm::MCAsmLayout &Layout) override {
        uint64_t offset = stream.tell();
        for (llvm::MCSection& sec : Asm) {
            assert(sec.getKind().isText() && "non-text sections not supported");
            Asm.writeSectionData(stream, &sec, Layout);
        }
        return stream.tell() - offset;
    }
};

struct HexBuffer {
    char* buf;
    size_t size;
    friend std::ostream& operator<<(std::ostream& os, HexBuffer const& self) {
        os << std::hex << std::setfill('0');
        for (size_t i = 0; i != self.size; i++)
            os << std::setw(2) << static_cast<int>(static_cast<uint8_t>(self.buf[i]));
        return os << std::dec;
    }
};

int main(int argc, char** argv) {
    int retval = 0;

    std::string triplestr;
    std::string cpufeatures;
    if (argc != 2) {
        std::cerr << "usage: " << argv[0] << " [architecture]" << std::endl;
        return 1;
    }

    if (false) {
#ifdef TARGET_X86_64
    } else if (!strcmp(argv[1], "x86_64")) {
        triplestr = "x86_64-linux-gnu";
        cpufeatures = "+nopl";
        LLVMInitializeX86TargetInfo();
        LLVMInitializeX86Target();
        LLVMInitializeX86TargetMC();
        LLVMInitializeX86AsmParser();
#endif // TARGET_X86_64
#ifdef TARGET_RV64
    } else if (!strcmp(argv[1], "rv64")) {
        triplestr = "riscv64-unknown-linux-gnu";
        cpufeatures = "+m,+a,+f,+d,+c";
        LLVMInitializeRISCVTargetInfo();
        LLVMInitializeRISCVTarget();
        LLVMInitializeRISCVTargetMC();
        LLVMInitializeRISCVAsmParser();
#endif // TARGET_RV64
    } else {
        std::cerr << "unsupported architecture" << std::endl;
        return 1;
    }

    std::string error;
    llvm::Triple triple(triplestr);
    const llvm::Target* target = llvm::TargetRegistry::lookupTarget("", triple, error);
    if (target == nullptr) {
        std::cerr << "error getting target: " << error << std::endl;
        return 1;
    }

    llvm::MCRegisterInfo* mri = target->createMCRegInfo(triple.str());
    if (mri == nullptr) {
        std::cerr << "error getting MCRegisterInfo" << std::endl;
        return 1;
    }

    llvm::MCTargetOptions options;
#if LL_LLVM_MAJOR < 10
    llvm::MCAsmInfo* mai = target->createMCAsmInfo(*mri, triple.str());
#else
    llvm::MCAsmInfo* mai = target->createMCAsmInfo(*mri, triple.str(), options);
#endif
    if (mai == nullptr) {
        std::cerr << "error getting MCAsmInfo" << std::endl;
        return 1;
    }

    llvm::MCInstrInfo* mcii = target->createMCInstrInfo();
    if (mcii == nullptr) {
        std::cerr << "error getting MCInstrInfo" << std::endl;
        return 1;
    }

    llvm::MCSubtargetInfo* sti = target->createMCSubtargetInfo(triple.str(), "", cpufeatures);
    if (sti == nullptr) {
        std::cerr << "error getting MCSubtargetInfo" << std::endl;
        return 1;
    }

    for (std::string asmline; std::getline(std::cin, asmline);) {
        if (asmline.rfind("!ASM", 0) != 0) {
            std::cout << asmline << std::endl;
            continue;
        }

        llvm::SourceMgr srcmgr;
        llvm::MCObjectFileInfo mofi;

        llvm::StringRef asmline_ref = asmline;
        std::unique_ptr<llvm::MemoryBuffer> asmbuf = llvm::MemoryBuffer::getMemBuffer(asmline_ref.drop_front(4));
        srcmgr.AddNewSourceBuffer(std::move(asmbuf), llvm::SMLoc());

        llvm::MCContext ctx(mai, mri, &mofi, &srcmgr);
        mofi.InitMCObjectFileInfo(triple, true, ctx);

        auto mab = std::unique_ptr<llvm::MCAsmBackend>(target->createMCAsmBackend(*sti, *mri, options));
        if (mab == nullptr) {
            std::cerr << "error getting MCAsmBackend" << std::endl;
            return 1;
        }

        auto mce = std::unique_ptr<llvm::MCCodeEmitter>(target->createMCCodeEmitter(*mcii, *mri, ctx));
        if (mce == nullptr) {
            std::cerr << "error getting MCCodeEmitter" << std::endl;
            return 1;
        }

        llvm::SmallVector<char, 1024> buf;
        llvm::raw_svector_ostream os(buf);
        auto ow = std::make_unique<PlainObjectWriter>(os);
        if (ow == nullptr) {
            std::cerr << "error getting MCObjectWriter" << std::endl;
            return 1;
        }

        llvm::MCStreamer* streamer = target->createMCObjectStreamer(triple, ctx,
                            std::move(mab), std::move(ow), std::move(mce), *sti,
                            options.MCRelaxAll, false, false);
        if (streamer == nullptr) {
            std::cerr << "error getting MCObjectStreamer" << std::endl;
            return 1;
        }

        llvm::MCAsmParser* map = llvm::createMCAsmParser(srcmgr, ctx, *streamer,
                                                         *mai);
        if (map == nullptr) {
            std::cerr << "error getting MCAsmParser" << std::endl;
            return 1;
        }

        llvm::MCTargetAsmParser* tap = target->createMCAsmParser(*sti, *map,
                                                                 *mcii,
                                                                 options);
        if (map == nullptr) {
            std::cerr << "error getting MCTargetAsmParser" << std::endl;
            return 1;
        }

        map->setTargetParser(*tap);
        if (map->Run(false, false)) {
            std::cout << "!" << std::endl;
            retval = 1;
        } else {
            std::cout << HexBuffer{buf.data(), buf.size()} << std::endl;
        }

        delete tap;
        delete map;
        delete streamer;
    }

    return retval;
}
