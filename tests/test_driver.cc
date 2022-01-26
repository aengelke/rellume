
#include <rellume/rellume.h>

#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/GenericValue.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/TargetSelect.h>

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <sys/mman.h>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>


#ifndef MAP_FIXED_NOREPLACE
#define MAP_FIXED_NOREPLACE 0x100000
#endif

static bool opt_verbose = false;
static bool opt_jit = false;
static bool opt_overflow_intrinsics = false;
static const char* opt_arch = "x86_64";

struct HexBuffer {
    uint8_t* buf;
    size_t size;
    friend std::ostream& operator<<(std::ostream& os, HexBuffer const& self) {
        os << std::hex << std::setfill('0');
        for (size_t i = 0; i != self.size; i++)
            os << std::setw(2) << static_cast<int>(self.buf[i]);
        return os << std::dec;
    }
};

struct CPU {
    uint8_t rip[8];
    uint8_t data[4096-8];
} __attribute__((aligned(64)));

struct RegEntry {
    size_t size;
    off_t offset;
};

class TestCase {
    const std::unordered_map<std::string,RegEntry>* regs;
    std::ostringstream& diagnostic;
    std::vector<std::pair<void*, size_t>> mem_maps;

    TestCase(std::ostringstream& diagnostic) : diagnostic(diagnostic) {
        static std::unordered_map<std::string,RegEntry> regs_empty = {};
#ifdef RELLUME_WITH_X86_64
        static std::unordered_map<std::string,RegEntry> regs_x86_64 = {
#define RELLUME_NAMED_REG(name,nameu,sz,off) {#name, {sz, off}},
#include <rellume/cpustruct-x86_64-private.inc>
#undef RELLUME_NAMED_REG
        };
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        static std::unordered_map<std::string,RegEntry> regs_rv64 = {
#define RELLUME_NAMED_REG(name,nameu,sz,off) {#name, {sz, off}},
#include <rellume/cpustruct-rv64-private.inc>
#undef RELLUME_NAMED_REG
        };
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        static std::unordered_map<std::string,RegEntry> regs_aarch64 = {
#define RELLUME_NAMED_REG(name,nameu,sz,off) {#name, {sz, off}},
#include <rellume/cpustruct-aarch64-private.inc>
#undef RELLUME_NAMED_REG
        };
#endif // RELLUME_WITH_AARCH64

        regs = &regs_empty;
#ifdef RELLUME_WITH_X86_64
        if (!strcmp(opt_arch, "x86_64"))
            regs = &regs_x86_64;
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
        if (!strcmp(opt_arch, "rv64"))
            regs = &regs_rv64;
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
        if (!strcmp(opt_arch, "aarch64"))
            regs = &regs_aarch64;
#endif // RELLUME_WITH_AARCH64
    }

    ~TestCase() {
        for (auto& map : mem_maps) {
            munmap(map.first, map.second);
        }
    }

    bool SetReg(std::string reg, std::string value_str, CPU* cpu) {
        auto reg_entry = regs->find(reg);
        if (reg_entry == regs->end()) {
            diagnostic << "# invalid register: " << reg << std::endl;
            return true;
        }

        if (value_str.length() != reg_entry->second.size * 2) {
            diagnostic << "# invalid input length: " << value_str << std::endl;
            return true;
        }

        uint8_t* cpu_raw = reinterpret_cast<uint8_t*>(cpu);
        uint8_t* buf = cpu_raw + reg_entry->second.offset;
        for (size_t i = 0; i < reg_entry->second.size; i++) {
            char hex_byte[3] = {value_str[i*2],value_str[i*2+1], 0};
            buf[i] = std::strtoul(hex_byte, nullptr, 16);
        }

        return false;
    }

    bool AllocMem(std::string key, std::string value_str) {
        uintptr_t addr = std::stoul(key.substr(1), nullptr, 16);
        size_t value_len = value_str.length() / 2;

        uintptr_t paged_addr = addr & -sysconf(_SC_PAGE_SIZE);
        size_t paged_size = value_len + (addr - paged_addr);
        void* map = mmap(reinterpret_cast<void*>(paged_addr), paged_size,
                         PROT_READ|PROT_WRITE,
                         MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE, -1, 0);
        if (map == MAP_FAILED || reinterpret_cast<uintptr_t>(map) != paged_addr) {
            diagnostic << "# error mapping address " << std::hex << addr << std::endl;
            return true;
        }
        mem_maps.push_back(std::make_pair(map, paged_size));

        uint8_t* buf = reinterpret_cast<uint8_t*>(addr);
        for (size_t i = 0; i < value_len; i++) {
            char hex_byte[3] = {value_str[i*2],value_str[i*2+1], 0};
            buf[i] = std::strtoul(hex_byte, nullptr, 16);
        }

        return false;
    }

    bool CheckMem(std::string key, std::string value_str) {
        uintptr_t addr = std::stoul(key.substr(1), nullptr, 16);
        size_t value_len = value_str.length() / 2;

        uint8_t* buf = reinterpret_cast<uint8_t*>(addr);
        bool fail = false;
        for (size_t i = 0; i < value_len; i++) {
            char hex_byte[3] = {value_str[i*2],value_str[i*2+1], 0};
            uint8_t val = std::strtoul(hex_byte, nullptr, 16);
            if (buf[i] != val) {
                fail = true;
                diagnostic << "# unexpected value for " << std::hex << addr << std::endl;
                diagnostic << "# expected: " << HexBuffer{&val, 1} << std::endl;
                diagnostic << "#      got: " << HexBuffer{&buf[i], 1} << std::endl;
            }
        }

        return fail;
    }

    std::pair<std::string, std::string> split_arg(std::string arg) {
        size_t value_off = arg.find('=');
        if (value_off == std::string::npos) {
            std::cerr << "invalid input: " << arg << std::endl;
            std::exit(1);
        }

        std::string key_str = arg.substr(0, value_off);
        std::string value_str = arg.substr(value_off + 1);
        return std::make_pair(key_str, value_str);
    }

    template<typename T>
    void Randomize(T& t) {
        using bytes_randomizer = std::independent_bits_engine<std::mt19937, CHAR_BIT, uint8_t>;
        std::mt19937 engine;
        bytes_randomizer rand_bytes(engine);

        uint8_t* ptr = reinterpret_cast<uint8_t*>(&t);
        for (size_t i = 0; i < sizeof(T); i++)
            ptr[i] = rand_bytes();
    }

    bool Run(std::string argstring) {
        std::istringstream argstream(argstring);
        std::string arg;
        bool fail = false;
        bool should_pass = true;
        bool use_jit = opt_jit;

        // 1. Setup initial state
        CPU initial{};
        Randomize(initial);

        while (argstream >> arg) {
            if (arg == "!") {
                should_pass = false;
            } else if (arg == "+jit") {
                use_jit = true;
            } else if (arg == "-jit") {
                use_jit = false;
            } else if (arg == "=>") {
                goto run_function;
            } else {
                auto kv = split_arg(arg);
                if (kv.first[0] == 'm') {
                    AllocMem(kv.first, kv.second);
                } else {
                    SetReg(kv.first, kv.second, &initial);
                }
            }
        }

        // We didn't run anything.
        diagnostic << "# error: no emulation command" << std::endl;
        return true;

    run_function:

        // 2. Emulate function
        CPU state = initial;

        llvm::LLVMContext ctx;
        auto mod = std::make_unique<llvm::Module>("rellume_test", ctx);

        LLConfig* rlcfg = ll_config_new();
        ll_config_enable_verify_ir(rlcfg, true);
        ll_config_enable_overflow_intrinsics(rlcfg, opt_overflow_intrinsics);
        bool success = ll_config_set_architecture(rlcfg, opt_arch);
        if (!success) {
            diagnostic << "# error: unsupported architecture" << std::endl;
            return true;
        }

        LLFunc* rlfn = ll_func_new(llvm::wrap(mod.get()), rlcfg);
        bool decode_ok = !ll_func_decode_cfg(rlfn, *reinterpret_cast<uint64_t*>(&state.rip), nullptr, nullptr);
        LLVMValueRef fn_wrap = decode_ok ? ll_func_lift(rlfn) : nullptr;

        ll_func_dispose(rlfn);
        ll_config_free(rlcfg);

        if (!decode_ok) {
            diagnostic << "# error: could not handle first instruction" << std::endl;
            return should_pass;
        }
        if (!fn_wrap) {
            diagnostic << "# error during lifting" << std::endl;
            return should_pass;
        }

        llvm::Function* fn = llvm::unwrap<llvm::Function>(fn_wrap);
        fn->setName("test_function");
        if (opt_verbose)
            fn->print(llvm::errs());

        std::string error;

        llvm::TargetOptions options;
        options.EnableFastISel = true;

        llvm::EngineBuilder builder(std::move(mod));
        // There are two options: "Interpreter" and "JIT". Because we execute
        // the code once only, the interpreter is usually faster (even compared
        // to the -O0 JIT configuration).
        if (use_jit)
            builder.setEngineKind(llvm::EngineKind::JIT);
        else
            builder.setEngineKind(llvm::EngineKind::Interpreter);
        builder.setErrorStr(&error);
        builder.setOptLevel(llvm::CodeGenOpt::None);
        builder.setTargetOptions(options);

        if (llvm::ExecutionEngine* engine = builder.create()) {
            // If we have a JIT compiler, get address of compiled code.
            // Otherwise try to run the function using the interpreter.
            const auto& name = fn->getName();
            if (auto raw_ptr = engine->getFunctionAddress(name.str())) {
                auto fn_ptr = reinterpret_cast<void(*)(CPU*)>(raw_ptr);
                fn_ptr(&state);
            } else {
                engine->runFunction(fn, {llvm::PTOGV(&state)});
            }
            delete engine;
        } else {
            diagnostic << "# error creating engine: " << error << std::endl;
            return true;
        }

        // 3. Compare with expected values
        //  - memory is compared immediately
        //  - registers are compared separately to support undefined values
        CPU expected = initial;

        std::unordered_set<std::string> skip_regs;
        while (argstream >> arg) {
            auto kv = split_arg(arg);
            if (kv.first[0] == 'm') {
                fail |= CheckMem(kv.first, kv.second);
            } else if (kv.second == "undef") {
                skip_regs.insert(kv.first);
            } else {
                SetReg(kv.first, kv.second, &expected);
            }
        }

        uint8_t* state_raw = reinterpret_cast<uint8_t*>(&state);
        uint8_t* expected_raw = reinterpret_cast<uint8_t*>(&expected);
        for (const auto& reg_entry : *regs) {
            if (skip_regs.count(reg_entry.first) > 0)
                continue;

            size_t size = reg_entry.second.size;
            size_t offset = reg_entry.second.offset;
            uint8_t* expected_bytes = expected_raw + offset;
            uint8_t* state_bytes = state_raw + offset;
            if (memcmp(state_bytes, expected_bytes, size) != 0) {
                fail = true;
                diagnostic << "# unexpected value for " << reg_entry.first << std::endl;
                diagnostic << "# expected: " << HexBuffer{expected_bytes, size} << std::endl;
                diagnostic << "#      got: " << HexBuffer{state_bytes, size} << std::endl;
            }
        }

        return should_pass ? fail : !fail;
    }

public:
    static bool Run(unsigned number, std::string caseline,
                    std::ostream& output) {
        std::ostringstream diagnostic;
        TestCase test_case(diagnostic);
        bool fail = test_case.Run(caseline);
        if (fail)
            output << "not ";
        output << "ok " << number << " " << caseline << std::endl;
        output << diagnostic.str();
        return fail;
    }
};

int main(int argc, char** argv) {
    int opt;
    while ((opt = getopt(argc, argv, "vjiA:")) != -1) {
        switch (opt) {
        case 'v': opt_verbose = true; break;
        case 'j': opt_jit = true; break;
        case 'i': opt_overflow_intrinsics = true; break;
        case 'A': opt_arch = optarg; break;
        default:
usage:
            std::cerr << "usage: " << argv[0] << " [-v] [-j] [-i] [-A arch] casefile" << std::endl;
            return 1;
        }
    }

    if (optind >= argc)
        goto usage;

    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();

    std::ifstream casefile(argv[optind]);
    if (casefile.fail()) {
        std::cerr << "error opening casefile" << std::endl;
        return 1;
    }

    std::ostringstream output;
    unsigned count = 0;
    bool fail = false;
    for (std::string caseline; std::getline(casefile, caseline); count++)
        fail |= TestCase::Run(count + 1, caseline, output);

    std::cout << output.str() << "1.." << count << std::endl;

    return fail ? 1 : 0;
}
