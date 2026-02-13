/*
 *  QEMU MIPS emulation: Special opcode helpers
 *
 *  Copyright (c) 2004-2005 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "exec/translation-block.h"
#include "internal.h"

/* Specials */
target_ulong helper_di(CPUMIPSState *env)
{
    target_ulong t0 = env->CP0_Status;

    env->CP0_Status = t0 & ~(1 << CP0St_IE);
    return t0;
}

target_ulong helper_ei(CPUMIPSState *env)
{
    target_ulong t0 = env->CP0_Status;

    env->CP0_Status = t0 | (1 << CP0St_IE);
    return t0;
}

static void debug_pre_eret(CPUMIPSState *env)
{
    if (qemu_loglevel_mask(CPU_LOG_EXEC)) {
        qemu_log("ERET: PC " TARGET_FMT_lx " EPC " TARGET_FMT_lx,
                env->active_tc.PC, env->CP0_EPC);
        if (env->CP0_Status & (1 << CP0St_ERL)) {
            qemu_log(" ErrorEPC " TARGET_FMT_lx, env->CP0_ErrorEPC);
        }
        if (env->hflags & MIPS_HFLAG_DM) {
            qemu_log(" DEPC " TARGET_FMT_lx, env->CP0_DEPC);
        }
        qemu_log("\n");
    }
}

static void debug_post_eret(CPUMIPSState *env)
{
    if (qemu_loglevel_mask(CPU_LOG_EXEC)) {
        qemu_log("  =>  PC " TARGET_FMT_lx " EPC " TARGET_FMT_lx,
                env->active_tc.PC, env->CP0_EPC);
        if (env->CP0_Status & (1 << CP0St_ERL)) {
            qemu_log(" ErrorEPC " TARGET_FMT_lx, env->CP0_ErrorEPC);
        }
        if (env->hflags & MIPS_HFLAG_DM) {
            qemu_log(" DEPC " TARGET_FMT_lx, env->CP0_DEPC);
        }
        switch (mips_env_mmu_index(env)) {
        case 3:
            qemu_log(", ERL\n");
            break;
        case MIPS_HFLAG_UM:
            qemu_log(", UM\n");
            break;
        case MIPS_HFLAG_SM:
            qemu_log(", SM\n");
            break;
        case MIPS_HFLAG_KM:
            qemu_log("\n");
            break;
        default:
            cpu_abort(env_cpu(env), "Invalid MMU mode!\n");
            break;
        }
    }
}

bool mips_io_recompile_replay_branch(CPUState *cs, const TranslationBlock *tb)
{
    CPUMIPSState *env = cpu_env(cs);

    if ((env->hflags & MIPS_HFLAG_BMASK) != 0
        && !tcg_cflags_has(cs, CF_PCREL) && env->active_tc.PC != tb->pc) {
        env->active_tc.PC -= (env->hflags & MIPS_HFLAG_B16 ? 2 : 4);
        env->hflags &= ~MIPS_HFLAG_BMASK;
        return true;
    }
    return false;
}

static inline void exception_return(CPUMIPSState *env)
{
    debug_pre_eret(env);
    if (env->CP0_Status & (1 << CP0St_ERL)) {
        mips_env_set_pc(env, env->CP0_ErrorEPC);
        env->CP0_Status &= ~(1 << CP0St_ERL);
    } else {
        mips_env_set_pc(env, env->CP0_EPC);
        env->CP0_Status &= ~(1 << CP0St_EXL);
    }
    compute_hflags(env);
    debug_post_eret(env);
}

void helper_eret(CPUMIPSState *env)
{
    exception_return(env);
    env->CP0_LLAddr = 1;
    env->lladdr = 1;
}

void helper_eretnc(CPUMIPSState *env)
{
    exception_return(env);
}

void helper_deret(CPUMIPSState *env)
{
    debug_pre_eret(env);

    env->hflags &= ~MIPS_HFLAG_DM;
    compute_hflags(env);

    mips_env_set_pc(env, env->CP0_DEPC);

    debug_post_eret(env);
}

void helper_cache(CPUMIPSState *env, target_ulong addr, uint32_t op)
{
    static const char *const type_name[] = {
        "Primary Instruction",
        "Primary Data or Unified Primary",
        "Tertiary",
        "Secondary"
    };
    uint32_t cache_type = extract32(op, 0, 2);
    uint32_t cache_operation = extract32(op, 2, 3);

    /*
     * Secondary cache diagnostic operations (R10000).
     *
     * The PROM uses CACHE instructions targeting the secondary cache
     * (cache_type == 3) to test L2 tag and data SRAM.  The address
     * format for Index operations is:
     *   bit 0:              way select (2-way set-associative)
     *   bits [line_shift-1:3]: doubleword within line (for data ops)
     *   bits [size_bits-1:line_shift]: set index
     *
     * TagLo:TagHi serve as a 64-bit data register for transfers.
     * CP0_ErrCtl (register 26) carries ECC values for data operations.
     */
    if (cache_type == 3 && env->scache_size > 0) {
        uint32_t line_size = env->scache_line_size;
        uint32_t num_lines = env->scache_size / line_size;
        uint32_t dwords_per_line = line_size / 8;
        uint32_t way = addr & 1;
        uint32_t index = addr & (env->scache_size - 1);
        uint32_t set_idx = (index >> ctz32(line_size)) & ((num_lines / 2) - 1);
        uint32_t line_idx = (set_idx << 1) | way;
        uint32_t dword = (index >> 3) & (dwords_per_line - 1);

        switch (cache_operation) {
        case 1: /* Index Load Tag */
            env->CP0_TagLo = (uint32_t)env->scache_tag[line_idx];
            env->CP0_TagHi = env->scache_tag[line_idx] >> 32;
            break;
        case 2: /* Index Store Tag */
            env->scache_tag[line_idx] =
                ((uint64_t)(uint32_t)env->CP0_TagHi << 32) |
                (uint32_t)env->CP0_TagLo;
            break;
        case 6: /* Index Load Data */
            {
                uint32_t di = line_idx * dwords_per_line + dword;
                env->CP0_TagLo = (uint32_t)env->scache_data[di];
                env->CP0_TagHi = env->scache_data[di] >> 32;
                env->CP0_ErrCtl = env->scache_ecc[di];
            }
            break;
        case 7: /* Index Store Data */
            {
                uint32_t di = line_idx * dwords_per_line + dword;
                env->scache_data[di] =
                    ((uint64_t)(uint32_t)env->CP0_TagHi << 32) |
                    (uint32_t)env->CP0_TagLo;
                env->scache_ecc[di] = env->CP0_ErrCtl & 0xFFFF;
            }
            break;
        default:
            /* Other operations (invalidate, writeback, etc.) are NOPs */
            break;
        }
        return;
    }

    /* ITC cache operations */
    if (env->itc_tag) {
        target_ulong itc_index = addr & 0x1fffffff;

        switch (cache_operation) {
        case 0b010: /* Index Store Tag */
            memory_region_dispatch_write(env->itc_tag, itc_index,
                                         env->CP0_TagLo,
                                         MO_64, MEMTXATTRS_UNSPECIFIED);
            break;
        case 0b001: /* Index Load Tag */
            memory_region_dispatch_read(env->itc_tag, itc_index,
                                        &env->CP0_TagLo,
                                        MO_64, MEMTXATTRS_UNSPECIFIED);
            break;
        case 0b000: /* Index Invalidate */
        case 0b100: /* Hit Invalidate */
        case 0b110: /* Hit Writeback */
            /* no-op */
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "cache operation:%u (type: %s cache)\n",
                          cache_operation, type_name[cache_type]);
            break;
        }
        return;
    }

    /* No special cache handling - treat as NOP */
}
