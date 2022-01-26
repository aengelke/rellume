/**
 * This file is part of Rellume.
 *
 * (c) 2020, Alexis Engelke <alexis.engelke@tum.de>
 * (c) 2020, Dominik Okwieka <dominik.okwieka@t-online.de>
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
 * \file enum Arch definition; new header file to resolve cyclical include.
 **/

#ifndef RELLUME_ARCH_H
#define RELLUME_ARCH_H

namespace rellume {

enum class Arch : unsigned {
    INVALID,
#ifdef RELLUME_WITH_X86_64
    X86_64, // x86-64 (Intel 64, actually)
#endif // RELLUME_WITH_X86_64
#ifdef RELLUME_WITH_RV64
    RV64, // RISC-V, 64 bit
#endif // RELLUME_WITH_RV64
#ifdef RELLUME_WITH_AARCH64
    AArch64, // ARM AArch64
#endif // RELLUME_WITH_AARCH64

    // Backwards compatibility: if no architecture is specified explicitly,
    // Rellume defaults to x86-64.
#ifdef RELLUME_WITH_X86_64
    DEFAULT = X86_64
#else // !RELLUME_WITH_X86_64
    DEFAULT = INVALID
#endif // RELLUME_WITH_X86_64
};

} // namespace rellume

#endif
