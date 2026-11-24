//! Functional CPU08 (HCS08) emulator + two flash-controller models, so the vendored
//! flash routines run as they would on silicon and cross-check the driver ABI:
//! - classic: the MC9S08 FCMD controller (FCDIV/FSTAT/FCMD, write-latch-launch);
//! - s08p: the S08P (PA/PT/PL/PB) FCCOB command-queue controller at 0x3020.
//! The core implements the whole HCS08 instruction set (the PTxx/PBxx routines are
//! compiled C). Refs: HCS08RMV1 (CPU08 instruction set), MC9S08 RM, MC9S08PA60RM ch.4.

const std = @import("std");

// CCR masks (bits 6,5 read as 1, not modeled).
const C: u8 = 0x01;
const Z: u8 = 0x02;
const N: u8 = 0x04;
const I: u8 = 0x08;
const H: u8 = 0x10;
const V: u8 = 0x80;

// Classic MC9S08 flash controller.
const FCDIV = 0;
const FSTAT = 5;
const FCMD = 6;
const FCBEF: u8 = 0x80;
const FCCF: u8 = 0x40;
const FPVIOL: u8 = 0x20;
const FACCERR: u8 = 0x10;
const CMD_BLANK: u8 = 0x05;
const CMD_BYTE_PROG: u8 = 0x20;
const CMD_BURST_PROG: u8 = 0x25;
const CMD_PAGE_ERASE: u8 = 0x40;
const CMD_MASS_ERASE: u8 = 0x41;

// S08P NVM (FCCOB) controller: offsets from its base and bits (MC9S08PA60RM 4.6).
pub const P_FCLKDIV = 0x0;
pub const P_FSEC = 0x1;
pub const P_FCCOBIX = 0x2;
pub const P_FCNFG = 0x4;
pub const P_FERCNFG = 0x5;
pub const P_FSTAT = 0x6;
pub const P_FERSTAT = 0x7;
pub const P_FPROT = 0x8;
pub const P_EEPROT = 0x9;
pub const P_FCCOBHI = 0xA;
pub const P_FCCOBLO = 0xB;
pub const P_FOPT = 0xC;
const P_LAST = P_FOPT;
pub const P_CCIF: u8 = 0x80;
pub const P_ACCERR: u8 = 0x20;
pub const P_FPVIOL: u8 = 0x10;
pub const P_MGSTAT0: u8 = 0x01;
const P_CMD_ERASE_VERIFY_ALL: u8 = 0x01;
const P_CMD_PROGRAM_FLASH: u8 = 0x06;
const P_CMD_ERASE_ALL_BLOCKS: u8 = 0x08;
const P_CMD_ERASE_FLASH_BLOCK: u8 = 0x09;
const P_CMD_ERASE_FLASH_SECTOR: u8 = 0x0A;
const P_CMD_UNSECURE: u8 = 0x0B;
const P_CMD_PROGRAM_EEPROM: u8 = 0x11;
const P_CMD_ERASE_EEPROM_SECTOR: u8 = 0x12;
/// FCCOB0 low byte flag selecting the EEPROM block (Erase Flash Block).
const P_EEPROM_BLOCK_FLAG: u8 = 0x80;
const P_FLASH_SECTOR: u32 = 512;
const P_EEPROM_SECTOR: u32 = 2;
/// NV_FSEC: copied into FSEC at reset.
const P_NV_FSEC: u16 = 0xFF7F;

/// BDCSCR.BDMACT: set when self-halted (BGND).
pub const BDMACT: u8 = 0x40;

pub const Error = error{ UnimplementedOpcode, Runaway, IllegalAddress } || std.mem.Allocator.Error;

pub const Range = struct { lo: u32, hi: u32 }; // inclusive

/// Geometry of an S08P part for its NVM model: up to two flash ranges (PA60 splits
/// around the 0x30xx registers and EEPROM) and the EEPROM.
pub const S08pLayout = struct {
    ctrl_base: u32 = 0x3020,
    flash: [2]?Range,
    eeprom: ?Range = null,
    /// Last RAM address. When set, CPU accesses outside the implemented map
    /// (0x0000..ram_hi, the 0x3000..0x30FF high registers, EEPROM, flash) are an
    /// illegal-address (ILAD) reset on the silicon: `step` fails with IllegalAddress.
    ram_hi: ?u32 = null,
};

const Nvm = enum { classic, s08p };

/// Live state of the S08P NVM controller.
const S08pRegs = struct {
    fclkdiv: u8 = 0,
    fsec: u8 = 0xFE,
    fccobix: u8 = 0,
    fccob: [6]u16 = [_]u16{0} ** 6,
    fcnfg: u8 = 0,
    fercnfg: u8 = 0,
    fstat: u8 = P_CCIF,
    ferstat: u8 = 0,
    fprot: u8 = 0xFF,
    eeprot: u8 = 0xFF,
    fopt: u8 = 0xFF,
};

pub const Cpu = struct {
    gpa: std.mem.Allocator,
    mem: []u8,
    a: u8 = 0,
    h: u8 = 0,
    x: u8 = 0,
    sp: u16 = 0x00FF,
    pc: u16 = 0,
    ccr: u8 = I,
    halted: bool = false,

    nvm: Nvm = .classic,
    ctrl_base: u32,
    flash_lo: u32,
    flash_hi: u32,
    fstat: u8 = FCBEF | FCCF,
    fcmd: u8 = 0,
    /// FCDIV: bit 7 DIVLD is set by the first write after reset, which also sets the
    /// divider; later writes are ignored until the next reset (as the silicon does).
    fcdiv: u8 = 0,
    /// Model the flash controller refusing commands (FACCERR) until FCDIV is loaded.
    /// Off for tests that drive the routine directly; on for the --sim CLI path, where
    /// forgetting to load it is exactly the bug worth catching. (The S08P model always
    /// enforces FCLKDIV, as its RM states.)
    require_fcdiv: bool = false,
    latched: bool = false,
    latch_addr: u16 = 0,
    latch_data: u8 = 0,

    p: S08pRegs = .{},
    p_layout: S08pLayout = .{ .flash = .{ null, null } },
    /// Commands launched (S08P), for tests.
    p_commands: u32 = 0,

    bad_opcode: u8 = 0,
    bad_pc: u16 = 0,
    /// S08P ILAD model (see `S08pLayout.ram_hi`): the first illegal address a CPU
    /// access touched, and the PC of that instruction.
    ilad_addr: ?u16 = null,
    ilad_pc: u16 = 0,
    stepping: bool = false,
    step_pc: u16 = 0,

    pub fn init(gpa: std.mem.Allocator, ctrl_base: u32, flash_lo: u32, flash_hi: u32) Error!Cpu {
        const mem = try gpa.alloc(u8, 0x10000);
        @memset(mem, 0);
        // erased flash reads 0xFF
        const lo: usize = flash_lo;
        const end: usize = @min(@as(usize, flash_hi) + 1, mem.len);
        if (lo < end) @memset(mem[lo..end], 0xFF);
        return .{ .gpa = gpa, .mem = mem, .ctrl_base = ctrl_base, .flash_lo = flash_lo, .flash_hi = flash_hi };
    }

    /// An S08P part: FCCOB controller at `layout.ctrl_base`, erased flash + EEPROM,
    /// unsecured (NV_FSEC programmed 0xFE so a reset keeps it that way).
    pub fn initS08p(gpa: std.mem.Allocator, layout: S08pLayout) Error!Cpu {
        const mem = try gpa.alloc(u8, 0x10000);
        @memset(mem, 0);
        var lo: u32 = 0xFFFF;
        var hi: u32 = 0;
        for (layout.flash) |r| if (r) |rg| {
            @memset(mem[rg.lo .. rg.hi + 1], 0xFF);
            lo = @min(lo, rg.lo);
            hi = @max(hi, rg.hi);
        };
        if (layout.eeprom) |e| @memset(mem[e.lo .. e.hi + 1], 0xFF);
        mem[P_NV_FSEC] = 0xFE;
        var cpu: Cpu = .{ .gpa = gpa, .mem = mem, .ctrl_base = layout.ctrl_base, .flash_lo = lo, .flash_hi = hi, .nvm = .s08p, .p_layout = layout };
        cpu.resetFlashRegs();
        return cpu;
    }

    pub fn deinit(self: *Cpu) void {
        self.gpa.free(self.mem);
    }

    fn hx(self: *const Cpu) u16 {
        return (@as(u16, self.h) << 8) | self.x;
    }
    fn setHx(self: *Cpu, v: u16) void {
        self.h = @truncate(v >> 8);
        self.x = @truncate(v);
    }
    fn setFlag(self: *Cpu, mask: u8, on: bool) void {
        if (on) self.ccr |= mask else self.ccr &= ~mask;
    }
    fn flag(self: *const Cpu, mask: u8) bool {
        return self.ccr & mask != 0;
    }
    fn nz8(self: *Cpu, r: u8) void {
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
    }
    fn nz16(self: *Cpu, r: u16) void {
        self.setFlag(N, r & 0x8000 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, false);
    }

    fn inCtrl(self: *const Cpu, a: u32) bool {
        const last: u32 = if (self.nvm == .s08p) P_LAST else FCMD;
        return a >= self.ctrl_base and a <= self.ctrl_base + last;
    }

    fn checkIlad(self: *Cpu, a: u32) void {
        if (!self.stepping or self.nvm != .s08p or self.ilad_addr != null) return;
        const ram_hi = self.p_layout.ram_hi orelse return;
        if (a <= ram_hi or (a >= 0x3000 and a <= 0x30FF) or self.p08InFlash(a) or self.p08InEeprom(a)) return;
        self.ilad_addr = @intCast(a);
        self.ilad_pc = self.step_pc;
    }

    fn read(self: *Cpu, addr: u16) u8 {
        const a: u32 = addr;
        self.checkIlad(a);
        if (self.inCtrl(a)) {
            const off = a - self.ctrl_base;
            if (self.nvm == .s08p) return self.p08Read(off);
            return switch (off) {
                FCDIV => self.fcdiv,
                FSTAT => self.fstat,
                FCMD => self.fcmd,
                else => 0,
            };
        }
        return self.mem[addr];
    }
    fn read16(self: *Cpu, addr: u16) u16 {
        return (@as(u16, self.read(addr)) << 8) | self.read(addr +% 1);
    }
    fn write(self: *Cpu, addr: u16, val: u8) void {
        const a: u32 = addr;
        self.checkIlad(a);
        if (self.inCtrl(a)) {
            const off = a - self.ctrl_base;
            if (self.nvm == .s08p) self.p08Write(off, val) else self.flashRegWrite(@intCast(off), val);
            return;
        }
        if (self.nvm == .s08p) {
            // The FCCOB controller programs through its registers; CPU stores to the
            // arrays do nothing.
            if (self.p08InFlash(a) or self.p08InEeprom(a)) return;
            self.mem[addr] = val;
            return;
        }
        if (a >= self.flash_lo and a <= self.flash_hi) {
            // latch only; array commits at launch
            self.latch_addr = addr;
            self.latch_data = val;
            self.latched = true;
            return;
        }
        self.mem[addr] = val;
    }
    fn write16(self: *Cpu, addr: u16, v: u16) void {
        self.write(addr, @truncate(v >> 8));
        self.write(addr +% 1, @truncate(v));
    }

    // --- classic MC9S08 flash controller ----------------------------------------

    fn flashRegWrite(self: *Cpu, off: u8, val: u8) void {
        switch (off) {
            FCDIV => {
                if (self.fcdiv & 0x80 == 0) self.fcdiv = (val & 0x7F) | 0x80;
            },
            FCMD => self.fcmd = val,
            FSTAT => {
                if (val & FACCERR != 0) self.fstat &= ~FACCERR;
                if (val & FPVIOL != 0) self.fstat &= ~FPVIOL;
                if (val & FCBEF != 0 and self.latched) self.launch();
            },
            else => {}, // FOPT/FCNFG/FPROT: inert
        }
    }
    fn launch(self: *Cpu) void {
        if (self.require_fcdiv and self.fcdiv & 0x80 == 0) {
            // Command with FCDIV unloaded: access error, nothing happens.
            self.fstat = FCBEF | FCCF | FACCERR;
            self.latched = false;
            return;
        }
        switch (self.fcmd) {
            CMD_BYTE_PROG, CMD_BURST_PROG => {
                // program only clears bits; on erased flash a store
                self.mem[self.latch_addr] &= self.latch_data;
            },
            CMD_PAGE_ERASE => {
                const page: u32 = self.latch_addr & ~@as(u32, 0x1FF); // 512-byte page
                var p = page;
                while (p < page + 0x200 and p <= self.flash_hi) : (p += 1) {
                    if (p >= self.flash_lo) self.mem[@intCast(p)] = 0xFF;
                }
            },
            CMD_MASS_ERASE => {
                var p = self.flash_lo;
                while (p <= self.flash_hi and p <= 0xFFFF) : (p += 1) self.mem[@intCast(p)] = 0xFF;
            },
            CMD_BLANK => {}, // routine blank-checks via direct reads
            else => {},
        }
        self.fstat = FCBEF | FCCF; // success
        self.latched = false;
    }

    // --- S08P FCCOB controller ----------------------------------------------------

    fn p08InFlash(self: *const Cpu, a: u32) bool {
        for (self.p_layout.flash) |r| if (r) |rg| {
            if (a >= rg.lo and a <= rg.hi) return true;
        };
        return false;
    }
    fn p08InEeprom(self: *const Cpu, a: u32) bool {
        const e = self.p_layout.eeprom orelse return false;
        return a >= e.lo and a <= e.hi;
    }
    fn p08Read(self: *Cpu, off: u32) u8 {
        const r = &self.p;
        const ix = r.fccobix & 7;
        return switch (off) {
            P_FCLKDIV => r.fclkdiv,
            P_FSEC => r.fsec,
            P_FCCOBIX => r.fccobix,
            P_FCNFG => r.fcnfg,
            P_FERCNFG => r.fercnfg,
            P_FSTAT => r.fstat,
            P_FERSTAT => r.ferstat,
            P_FPROT => r.fprot,
            P_EEPROT => r.eeprot,
            P_FCCOBHI => if (ix < 6) @truncate(r.fccob[ix] >> 8) else 0,
            P_FCCOBLO => if (ix < 6) @truncate(r.fccob[ix]) else 0,
            P_FOPT => r.fopt,
            else => 0,
        };
    }
    fn p08Write(self: *Cpu, off: u32, val: u8) void {
        const r = &self.p;
        const ix = r.fccobix & 7;
        switch (off) {
            P_FCLKDIV => {
                // Writable while idle and unlocked; FDIVLD sets on any write.
                if (r.fstat & P_CCIF != 0 and r.fclkdiv & 0x40 == 0) r.fclkdiv = 0x80 | (val & 0x7F);
            },
            P_FCCOBIX => r.fccobix = val & 7,
            P_FCNFG => r.fcnfg = val,
            P_FERCNFG => r.fercnfg = val,
            P_FSTAT => {
                if (val & P_ACCERR != 0) r.fstat &= ~P_ACCERR;
                if (val & P_FPVIOL != 0) r.fstat &= ~P_FPVIOL;
                if (val & P_CCIF != 0 and r.fstat & P_CCIF != 0) self.p08Launch();
            },
            P_FERSTAT => r.ferstat &= ~val,
            P_FPROT => r.fprot = val,
            P_EEPROT => r.eeprot = val,
            P_FCCOBHI => if (ix < 6) {
                r.fccob[ix] = (r.fccob[ix] & 0x00FF) | (@as(u16, val) << 8);
            },
            P_FCCOBLO => if (ix < 6) {
                r.fccob[ix] = (r.fccob[ix] & 0xFF00) | val;
            },
            else => {}, // FSEC/FOPT read-only
        }
    }
    fn p08EraseRange(self: *Cpu, lo: u32, len: u32) void {
        var a = lo;
        while (a < lo + len and a <= 0xFFFF) : (a += 1) self.mem[@intCast(a)] = 0xFF;
    }
    fn p08EraseAll(self: *Cpu, flash: bool, eeprom: bool) void {
        if (flash) for (self.p_layout.flash) |r| if (r) |rg| self.p08EraseRange(rg.lo, rg.hi - rg.lo + 1);
        if (eeprom) if (self.p_layout.eeprom) |e| self.p08EraseRange(e.lo, e.hi - e.lo + 1);
    }
    fn p08AllErased(self: *Cpu) bool {
        for (self.p_layout.flash) |r| if (r) |rg| {
            for (self.mem[rg.lo .. rg.hi + 1]) |b| if (b != 0xFF) return false;
        };
        if (self.p_layout.eeprom) |e| for (self.mem[e.lo .. e.hi + 1]) |b| if (b != 0xFF) return false;
        return true;
    }
    fn p08Launch(self: *Cpu) void {
        const r = &self.p;
        self.p_commands += 1;
        r.fstat &= ~P_MGSTAT0;
        // RM 4.5.2.4.1: with FCLKDIV unwritten since reset nothing executes.
        if (r.fclkdiv & 0x80 == 0) {
            r.fstat |= P_ACCERR;
            return;
        }
        const cmd: u8 = @truncate(r.fccob[0] >> 8);
        const ix = r.fccobix & 7;
        const addr: u32 = r.fccob[1];
        const secured = r.fsec & 0x03 != 0x02;
        switch (cmd) {
            P_CMD_ERASE_ALL_BLOCKS, P_CMD_UNSECURE => {
                self.p08EraseAll(true, true);
                // Verified erased -> unsecured (FSEC SEC = 10) until the next reset.
                r.fsec = (r.fsec & ~@as(u8, 0x03)) | 0x02;
            },
            P_CMD_ERASE_VERIFY_ALL => if (!self.p08AllErased()) {
                r.fstat |= P_MGSTAT0;
            },
            P_CMD_ERASE_FLASH_BLOCK => {
                const ee = @as(u8, @truncate(r.fccob[0])) & P_EEPROM_BLOCK_FLAG != 0;
                self.p08EraseAll(!ee, ee);
            },
            P_CMD_ERASE_FLASH_SECTOR => {
                if (secured or !self.p08InFlash(addr) or ix != 1) {
                    r.fstat |= P_ACCERR;
                    return;
                }
                if (r.fprot != 0xFF) {
                    r.fstat |= P_FPVIOL;
                    return;
                }
                self.p08EraseRange(addr & ~(P_FLASH_SECTOR - 1), P_FLASH_SECTOR);
            },
            P_CMD_PROGRAM_FLASH => {
                // One or two longwords: FCCOB2..3 or FCCOB2..5, 4-byte aligned.
                if (secured or (ix != 3 and ix != 5) or addr & 3 != 0 or !self.p08InFlash(addr)) {
                    r.fstat |= P_ACCERR;
                    return;
                }
                const words: u32 = @as(u32, ix) - 1;
                if (!self.p08InFlash(addr + words * 2 - 1)) {
                    r.fstat |= P_ACCERR;
                    return;
                }
                if (r.fprot != 0xFF) {
                    r.fstat |= P_FPVIOL;
                    return;
                }
                var w: u32 = 0;
                while (w < words) : (w += 1) {
                    const a: u16 = @intCast(addr + w * 2);
                    const d = r.fccob[2 + w];
                    self.mem[a] &= @truncate(d >> 8); // program only clears bits
                    self.mem[a + 1] &= @truncate(d);
                }
            },
            P_CMD_PROGRAM_EEPROM => {
                // One to four bytes, in the low halves of FCCOB2..5.
                if (secured or ix < 2 or ix > 5 or !self.p08InEeprom(addr)) {
                    r.fstat |= P_ACCERR;
                    return;
                }
                const n: u32 = @as(u32, ix) - 1;
                if (!self.p08InEeprom(addr + n - 1)) {
                    r.fstat |= P_ACCERR;
                    return;
                }
                var i: u32 = 0;
                while (i < n) : (i += 1) self.mem[@intCast(addr + i)] &= @truncate(r.fccob[2 + i]);
            },
            P_CMD_ERASE_EEPROM_SECTOR => {
                if (secured or !self.p08InEeprom(addr) or ix != 1) {
                    r.fstat |= P_ACCERR;
                    return;
                }
                self.p08EraseRange(addr & ~(P_EEPROM_SECTOR - 1), P_EEPROM_SECTOR);
            },
            else => r.fstat |= P_ACCERR,
        }
    }

    /// Reset the flash controller as a target reset does: classic FCDIV unloaded
    /// (DIVLD clear); S08P FCLKDIV unloaded and FSEC reloaded from NV_FSEC.
    pub fn resetFlashRegs(self: *Cpu) void {
        self.fcdiv = 0;
        self.fstat = FCBEF | FCCF;
        self.latched = false;
        if (self.nvm == .s08p) {
            self.p = .{ .fsec = self.mem[P_NV_FSEC] };
        }
    }

    // BDM host access: direct to memory, but the flash controller's registers behave
    // like registers (FCDIV is write-once, FSTAT launches), not like RAM.
    pub fn hostWrite(self: *Cpu, addr: u16, data: []const u8) void {
        for (data, 0..) |b, i| {
            const a: u32 = @as(u32, addr) + @as(u32, @intCast(i));
            if (self.inCtrl(a)) {
                const off = a - self.ctrl_base;
                if (self.nvm == .s08p) self.p08Write(off, b) else self.flashRegWrite(@intCast(off), b);
            } else {
                self.mem[a] = b;
            }
        }
    }
    pub fn hostRead(self: *Cpu, addr: u16, out: []u8) void {
        for (out, 0..) |*b, i| b.* = self.read(@intCast(@as(u32, addr) + @as(u32, @intCast(i))));
    }

    // --- core ---------------------------------------------------------------------

    fn fetch8(self: *Cpu) u8 {
        self.checkIlad(self.pc);
        const b = self.mem[self.pc];
        self.pc +%= 1;
        return b;
    }
    fn fetch16(self: *Cpu) u16 {
        const hi = self.fetch8();
        const lo = self.fetch8();
        return (@as(u16, hi) << 8) | lo;
    }
    fn rel(self: *Cpu, taken: bool, offset: u8) void {
        const r: i8 = @bitCast(offset);
        if (taken) self.pc = @bitCast(@as(i16, @bitCast(self.pc)) +% r);
    }
    fn branch(self: *Cpu, taken: bool) void {
        self.rel(taken, self.fetch8());
    }
    fn push(self: *Cpu, v: u8) void {
        self.write(self.sp, v);
        self.sp -%= 1;
    }
    fn pull(self: *Cpu) u8 {
        self.sp +%= 1;
        return self.read(self.sp);
    }

    /// Branch condition for 0x20..0x2F and 0x90..0x93.
    fn cond(self: *const Cpu, op: u8) bool {
        const nv = self.flag(N) != self.flag(V);
        return switch (op) {
            0x20 => true, // BRA
            0x21 => false, // BRN
            0x22 => !self.flag(C) and !self.flag(Z), // BHI
            0x23 => self.flag(C) or self.flag(Z), // BLS
            0x24 => !self.flag(C), // BCC
            0x25 => self.flag(C), // BCS
            0x26 => !self.flag(Z), // BNE
            0x27 => self.flag(Z), // BEQ
            0x28 => !self.flag(H), // BHCC
            0x29 => self.flag(H), // BHCS
            0x2A => !self.flag(N), // BPL
            0x2B => self.flag(N), // BMI
            0x2C => !self.flag(I), // BMC
            0x2D => self.flag(I), // BMS
            0x2E => false, // BIL: IRQ pin modeled high
            0x2F => true, // BIH
            0x90 => !nv, // BGE
            0x91 => nv, // BLT
            0x92 => !self.flag(Z) and !nv, // BGT
            0x93 => self.flag(Z) or nv, // BLE
            else => false,
        };
    }

    // ALU on A (or X for CPX/LDX/STX) with operand `m`.
    fn sub(self: *Cpu, r_in: u8, m: u8, carry: bool) u8 {
        const cin: u8 = if (carry and self.flag(C)) 1 else 0;
        const full: i16 = @as(i16, r_in) - m - cin;
        const r: u8 = @truncate(@as(u16, @bitCast(full)));
        self.setFlag(C, full < 0);
        self.setFlag(V, (r_in ^ m) & (r_in ^ r) & 0x80 != 0);
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        return r;
    }
    fn cmp(self: *Cpu, m: u8) void {
        _ = self.sub(self.a, m, false);
    }
    fn add(self: *Cpu, m: u8, carry: bool) void {
        const cin: u16 = if (carry and self.flag(C)) 1 else 0;
        const r: u16 = @as(u16, self.a) + m + cin;
        const r8: u8 = @truncate(r);
        self.setFlag(H, ((self.a & 0xF) + (m & 0xF) + @as(u8, @intCast(cin))) > 0xF);
        self.setFlag(C, r > 0xFF);
        self.setFlag(V, (self.a ^ r8) & (m ^ r8) & 0x80 != 0);
        self.a = r8;
        self.setFlag(N, r8 & 0x80 != 0);
        self.setFlag(Z, r8 == 0);
    }
    fn cphx(self: *Cpu, m: u16) void {
        const v = self.hx();
        const r = v -% m;
        self.setFlag(C, v < m);
        self.setFlag(V, (v ^ m) & (v ^ r) & 0x8000 != 0);
        self.setFlag(N, r & 0x8000 != 0);
        self.setFlag(Z, r == 0);
    }

    /// The 0xA0..0xFF (and 9E D/E) ALU group, `lo` = low nibble, operand at `ea`
    /// (or the immediate byte for IMM). Returns false for an unused encoding.
    fn alu(self: *Cpu, lo: u8, ea: u16, imm: ?u8) bool {
        const m: u8 = if (imm) |b| b else if (lo == 0x7 or lo == 0xF or lo == 0xC or lo == 0xD) 0 else self.read(ea);
        switch (lo) {
            0x0 => self.a = self.sub(self.a, m, false), // SUB
            0x1 => self.cmp(m), // CMP
            0x2 => self.a = self.sub(self.a, m, true), // SBC
            0x3 => _ = self.sub(self.x, m, false), // CPX
            0x4 => { // AND
                self.a &= m;
                self.nz8(self.a);
            },
            0x5 => self.nz8(self.a & m), // BIT
            0x6 => { // LDA
                self.a = m;
                self.nz8(m);
            },
            0x7 => { // STA
                self.write(ea, self.a);
                self.nz8(self.a);
            },
            0x8 => { // EOR
                self.a ^= m;
                self.nz8(self.a);
            },
            0x9 => self.add(m, true), // ADC
            0xA => { // ORA
                self.a |= m;
                self.nz8(self.a);
            },
            0xB => self.add(m, false), // ADD
            0xC => self.pc = ea, // JMP
            0xD => { // JSR
                self.push(@truncate(self.pc));
                self.push(@truncate(self.pc >> 8));
                self.pc = ea;
            },
            0xE => { // LDX
                self.x = m;
                self.nz8(m);
            },
            0xF => { // STX
                self.write(ea, self.x);
                self.nz8(self.x);
            },
            else => return false,
        }
        return true;
    }

    /// Read-modify-write op `lo` on `v`; null for the non-RMW encodings (handled
    /// by the caller). TST returns v unchanged (caller skips the write).
    fn rmw(self: *Cpu, lo: u8, v: u8) ?u8 {
        switch (lo) {
            0x0 => { // NEG
                const r: u8 = 0 -% v;
                self.setFlag(V, r == 0x80);
                self.setFlag(C, r != 0);
                self.setFlag(N, r & 0x80 != 0);
                self.setFlag(Z, r == 0);
                return r;
            },
            0x3 => { // COM
                const r = ~v;
                self.nz8(r);
                self.setFlag(C, true);
                return r;
            },
            0x4 => { // LSR
                const r = v >> 1;
                self.setFlag(C, v & 1 != 0);
                self.setFlag(N, false);
                self.setFlag(Z, r == 0);
                self.setFlag(V, self.flag(C)); // N ^ C, N = 0
                return r;
            },
            0x6 => { // ROR
                const r = (v >> 1) | (if (self.flag(C)) @as(u8, 0x80) else 0);
                self.setFlag(C, v & 1 != 0);
                self.shiftFlags(r);
                return r;
            },
            0x7 => { // ASR
                const r = (v >> 1) | (v & 0x80);
                self.setFlag(C, v & 1 != 0);
                self.shiftFlags(r);
                return r;
            },
            0x8 => { // LSL
                const r = v << 1;
                self.setFlag(C, v & 0x80 != 0);
                self.shiftFlags(r);
                return r;
            },
            0x9 => { // ROL
                const r = (v << 1) | (if (self.flag(C)) @as(u8, 1) else 0);
                self.setFlag(C, v & 0x80 != 0);
                self.shiftFlags(r);
                return r;
            },
            0xA => { // DEC
                const r = v -% 1;
                self.setFlag(V, r == 0x7F);
                self.setFlag(N, r & 0x80 != 0);
                self.setFlag(Z, r == 0);
                return r;
            },
            0xC => { // INC
                const r = v +% 1;
                self.setFlag(V, r == 0x80);
                self.setFlag(N, r & 0x80 != 0);
                self.setFlag(Z, r == 0);
                return r;
            },
            0xD => { // TST
                self.nz8(v);
                return v;
            },
            0xF => { // CLR
                self.nz8(0);
                return 0;
            },
            else => return null,
        }
    }
    fn shiftFlags(self: *Cpu, r: u8) void {
        self.setFlag(N, r & 0x80 != 0);
        self.setFlag(Z, r == 0);
        self.setFlag(V, self.flag(N) != self.flag(C));
    }

    fn unimplemented(self: *Cpu, op: u8, at: u16) Error!bool {
        self.bad_opcode = op;
        self.bad_pc = at;
        return error.UnimplementedOpcode;
    }

    /// One instruction; false once self-halted (BGND).
    pub fn step(self: *Cpu) Error!bool {
        if (self.halted) return false;
        const at = self.pc;
        self.stepping = true;
        self.step_pc = at;
        defer {
            self.stepping = false;
        }
        const op = self.fetch8();
        const hi = op >> 4;
        const lo = op & 0x0F;
        switch (hi) {
            0x0 => { // BRSET/BRCLR n,dd,rel
                const dd = self.fetch8();
                const off = self.fetch8();
                const bit = (self.read(dd) >> @intCast(lo >> 1)) & 1 != 0;
                self.setFlag(C, bit);
                self.rel(if (lo & 1 == 0) bit else !bit, off);
            },
            0x1 => { // BSET/BCLR n,dd
                const dd = self.fetch8();
                const mask: u8 = @as(u8, 1) << @intCast(lo >> 1);
                const v = self.read(dd);
                self.write(dd, if (lo & 1 == 0) v | mask else v & ~mask);
            },
            0x2 => self.branch(self.cond(op)),
            0x3, 0x4, 0x5, 0x6, 0x7 => return self.rmwRow(op, at),
            0x8 => switch (op) {
                0x80 => { // RTI
                    self.ccr = self.pull();
                    self.a = self.pull();
                    self.x = self.pull();
                    const ph = self.pull();
                    const pl = self.pull();
                    self.pc = (@as(u16, ph) << 8) | pl;
                },
                0x81 => { // RTS
                    const ph = self.pull();
                    const pl = self.pull();
                    self.pc = (@as(u16, ph) << 8) | pl;
                },
                0x82 => { // BGND (self-halt)
                    self.halted = true;
                    return false;
                },
                0x83 => { // SWI
                    self.push(@truncate(self.pc));
                    self.push(@truncate(self.pc >> 8));
                    self.push(self.x);
                    self.push(self.a);
                    self.push(self.ccr);
                    self.setFlag(I, true);
                    self.pc = self.read16(0xFFFC);
                },
                0x84 => self.ccr = self.a, // TAP
                0x85 => self.a = self.ccr, // TPA
                0x86 => self.a = self.pull(), // PULA
                0x87 => self.push(self.a), // PSHA
                0x88 => self.x = self.pull(), // PULX
                0x89 => self.push(self.x), // PSHX
                0x8A => self.h = self.pull(), // PULH
                0x8B => self.push(self.h), // PSHH
                0x8C => self.h = 0, // CLRH
                0x8E, 0x8F => { // STOP / WAIT: nothing to wake it here
                    self.halted = true;
                    return false;
                },
                else => return self.unimplemented(op, at),
            },
            0x9 => switch (op) {
                0x90, 0x91, 0x92, 0x93 => self.branch(self.cond(op)),
                0x94 => self.sp = self.hx() -% 1, // TXS
                0x95 => self.setHx(self.sp +% 1), // TSX
                0x96 => { // STHX ext
                    const ea = self.fetch16();
                    self.write16(ea, self.hx());
                    self.nz16(self.hx());
                },
                0x97 => self.x = self.a, // TAX
                0x98 => self.setFlag(C, false), // CLC
                0x99 => self.setFlag(C, true), // SEC
                0x9A => self.setFlag(I, false), // CLI
                0x9B => self.setFlag(I, true), // SEI
                0x9C => self.sp = (self.sp & 0xFF00) | 0xFF, // RSP
                0x9D => {}, // NOP
                0x9E => return self.page2(at),
                0x9F => self.a = self.x, // TXA
                else => unreachable,
            },
            0xA => switch (lo) {
                0x7 => { // AIS #
                    const ii: i8 = @bitCast(self.fetch8());
                    self.sp = @bitCast(@as(i16, @bitCast(self.sp)) +% ii);
                },
                0xC => return self.unimplemented(op, at),
                0xD => { // BSR
                    const off = self.fetch8();
                    self.push(@truncate(self.pc));
                    self.push(@truncate(self.pc >> 8));
                    self.rel(true, off);
                },
                0xF => { // AIX #
                    const ii: i8 = @bitCast(self.fetch8());
                    self.setHx(@bitCast(@as(i16, @bitCast(self.hx())) +% ii));
                },
                else => _ = self.alu(lo, 0, self.fetch8()),
            },
            0xB => _ = self.alu(lo, self.fetch8(), null), // DIR
            0xC => _ = self.alu(lo, self.fetch16(), null), // EXT
            0xD => _ = self.alu(lo, self.hx() +% self.fetch16(), null), // IX2
            0xE => _ = self.alu(lo, self.hx() +% self.fetch8(), null), // IX1
            0xF => _ = self.alu(lo, self.hx(), null), // IX
            else => unreachable,
        }
        return true;
    }

    /// Rows 0x3..0x7: RMW on DIR / A / X / IX1 / IX, plus the odd encodings each
    /// row carries (CBEQ, DBNZ, MOV, LDHX/STHX/CPHX, MUL, DIV, NSA, DAA).
    fn rmwRow(self: *Cpu, op: u8, at: u16) Error!bool {
        const hi = op >> 4;
        const lo = op & 0x0F;
        switch (op) {
            0x31 => { // CBEQ dd,rel
                const m = self.read(self.fetch8());
                self.rel(self.a == m, self.fetch8());
            },
            0x41 => { // CBEQA #,rel
                const m = self.fetch8();
                self.rel(self.a == m, self.fetch8());
            },
            0x51 => { // CBEQX #,rel
                const m = self.fetch8();
                self.rel(self.x == m, self.fetch8());
            },
            0x61 => { // CBEQ ix1+,rel
                const m = self.read(self.hx() +% self.fetch8());
                self.setHx(self.hx() +% 1);
                self.rel(self.a == m, self.fetch8());
            },
            0x71 => { // CBEQ ,X+,rel
                const m = self.read(self.hx());
                self.setHx(self.hx() +% 1);
                self.rel(self.a == m, self.fetch8());
            },
            0x3B => { // DBNZ dd,rel
                const dd = self.fetch8();
                const r = self.read(dd) -% 1;
                self.write(dd, r);
                self.rel(r != 0, self.fetch8());
            },
            0x4B => { // DBNZA rel
                self.a -%= 1;
                self.rel(self.a != 0, self.fetch8());
            },
            0x5B => { // DBNZX rel
                self.x -%= 1;
                self.rel(self.x != 0, self.fetch8());
            },
            0x6B => { // DBNZ ix1,rel
                const ea = self.hx() +% self.fetch8();
                const r = self.read(ea) -% 1;
                self.write(ea, r);
                self.rel(r != 0, self.fetch8());
            },
            0x7B => { // DBNZ ,X,rel
                const ea = self.hx();
                const r = self.read(ea) -% 1;
                self.write(ea, r);
                self.rel(r != 0, self.fetch8());
            },
            0x4E => { // MOV dd,dd
                const v = self.read(self.fetch8());
                self.write(self.fetch8(), v);
                self.nz8(v);
            },
            0x5E => { // MOV dd,X+
                const v = self.read(self.fetch8());
                self.write(self.hx(), v);
                self.setHx(self.hx() +% 1);
                self.nz8(v);
            },
            0x6E => { // MOV #,dd
                const v = self.fetch8();
                self.write(self.fetch8(), v);
                self.nz8(v);
            },
            0x7E => { // MOV ,X+,dd
                const v = self.read(self.hx());
                self.setHx(self.hx() +% 1);
                self.write(self.fetch8(), v);
                self.nz8(v);
            },
            0x32 => { // LDHX ext
                self.setHx(self.read16(self.fetch16()));
                self.nz16(self.hx());
            },
            0x45 => { // LDHX #
                self.setHx(self.fetch16());
                self.nz16(self.hx());
            },
            0x55 => { // LDHX dd
                self.setHx(self.read16(self.fetch8()));
                self.nz16(self.hx());
            },
            0x35 => { // STHX dd
                self.write16(self.fetch8(), self.hx());
                self.nz16(self.hx());
            },
            0x3E => self.cphx(self.read16(self.fetch16())), // CPHX ext
            0x65 => self.cphx(self.fetch16()), // CPHX #
            0x75 => self.cphx(self.read16(self.fetch8())), // CPHX dd
            0x42 => { // MUL: X:A = X * A
                const r: u16 = @as(u16, self.x) * self.a;
                self.x = @truncate(r >> 8);
                self.a = @truncate(r);
                self.setFlag(H, false);
                self.setFlag(C, false);
            },
            0x52 => { // DIV: A = H:A / X, H = remainder
                const dividend: u16 = (@as(u16, self.h) << 8) | self.a;
                if (self.x == 0 or dividend / self.x > 0xFF) {
                    self.setFlag(C, true);
                } else {
                    self.a = @intCast(dividend / self.x);
                    self.h = @intCast(dividend % self.x);
                    self.setFlag(C, false);
                    self.setFlag(Z, self.a == 0);
                }
            },
            0x62 => self.a = (self.a << 4) | (self.a >> 4), // NSA
            0x72 => { // DAA
                var corr: u8 = 0;
                var carry = self.flag(C);
                if (self.flag(H) or (self.a & 0x0F) > 9) corr |= 0x06;
                if (carry or self.a > 0x99) {
                    corr |= 0x60;
                    carry = true;
                }
                self.a +%= corr;
                self.setFlag(N, self.a & 0x80 != 0);
                self.setFlag(Z, self.a == 0);
                self.setFlag(C, carry);
            },
            else => {
                switch (hi) {
                    0x4 => self.a = self.rmw(lo, self.a) orelse return self.unimplemented(op, at),
                    0x5 => self.x = self.rmw(lo, self.x) orelse return self.unimplemented(op, at),
                    else => {
                        const ea: u16 = switch (hi) {
                            0x3 => self.fetch8(),
                            0x6 => self.hx() +% self.fetch8(),
                            0x7 => self.hx(),
                            else => unreachable,
                        };
                        const r = self.rmw(lo, self.read(ea)) orelse return self.unimplemented(op, at);
                        if (lo != 0xD) self.write(ea, r); // TST doesn't write
                    },
                }
            },
        }
        return true;
    }

    /// 0x9E-prefixed page: SP-relative forms and the extra LDHX/STHX/CPHX modes.
    fn page2(self: *Cpu, at: u16) Error!bool {
        const op = self.fetch8();
        const hi = op >> 4;
        const lo = op & 0x0F;
        switch (op) {
            0xAE => { // LDHX ,X
                self.setHx(self.read16(self.hx()));
                self.nz16(self.hx());
            },
            0xBE => { // LDHX ix2
                self.setHx(self.read16(self.hx() +% self.fetch16()));
                self.nz16(self.hx());
            },
            0xCE => { // LDHX ix1
                self.setHx(self.read16(self.hx() +% self.fetch8()));
                self.nz16(self.hx());
            },
            0xFE => { // LDHX sp1
                self.setHx(self.read16(self.sp +% self.fetch8()));
                self.nz16(self.hx());
            },
            0xFF => { // STHX sp1
                self.write16(self.sp +% self.fetch8(), self.hx());
                self.nz16(self.hx());
            },
            0xF3 => self.cphx(self.read16(self.sp +% self.fetch8())), // CPHX sp1
            0x61 => { // CBEQ sp1,rel
                const m = self.read(self.sp +% self.fetch8());
                self.rel(self.a == m, self.fetch8());
            },
            0x6B => { // DBNZ sp1,rel
                const ea = self.sp +% self.fetch8();
                const r = self.read(ea) -% 1;
                self.write(ea, r);
                self.rel(r != 0, self.fetch8());
            },
            else => switch (hi) {
                0x6 => { // RMW sp1
                    const ea = self.sp +% self.fetch8();
                    const r = self.rmw(lo, self.read(ea)) orelse return self.unimplemented(op, at);
                    if (lo != 0xD) self.write(ea, r);
                },
                0xD => { // ALU sp2 (no JMP/JSR)
                    if (lo == 0xC or lo == 0xD) return self.unimplemented(op, at);
                    _ = self.alu(lo, self.sp +% self.fetch16(), null);
                },
                0xE => { // ALU sp1 (no JMP/JSR)
                    if (lo == 0xC or lo == 0xD) return self.unimplemented(op, at);
                    _ = self.alu(lo, self.sp +% self.fetch8(), null);
                },
                else => return self.unimplemented(op, at),
            },
        }
        return true;
    }

    /// Run until BGND (self-halt) or step cap.
    pub fn run(self: *Cpu, max_steps: usize) Error!void {
        self.halted = false;
        var n: usize = 0;
        while (n < max_steps) : (n += 1) {
            const more = try self.step();
            if (self.ilad_addr != null) return error.IllegalAddress;
            if (!more) return;
        }
        return error.Runaway;
    }
};

const testing = std.testing;

fn testCpu() !Cpu {
    return Cpu.init(testing.allocator, 0x1820, 0xE000, 0xFFFF);
}

fn runProg(cpu: *Cpu, at: u16, prog: []const u8) !void {
    cpu.hostWrite(at, prog);
    cpu.pc = at;
    try cpu.run(10_000);
}

test "core: LDA#/STA-dir/LDHX#/AIX/BEQ and self-halt on BGND" {
    var cpu = try testCpu();
    defer cpu.deinit();
    // LDHX #$0200 ; LDA #$AB ; STA $10 ; AIX #4 ; BGND
    const prog = [_]u8{ 0x45, 0x02, 0x00, 0xA6, 0xAB, 0xB7, 0x10, 0xAF, 0x04, 0x82 };
    cpu.hostWrite(0x0300, &prog);
    cpu.pc = 0x0300;
    try cpu.run(1000);
    try testing.expect(cpu.halted);
    try testing.expectEqual(@as(u8, 0xAB), cpu.mem[0x10]);
    try testing.expectEqual(@as(u16, 0x0204), cpu.hx());
    try testing.expectEqual(@as(u8, BDMACT), if (cpu.halted) BDMACT else 0);
}

test "flash controller: program latches then commits on FCBEF launch" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.mem[0xE000] = 0xFF; // erased
    cpu.write(0xE000, 0x5A); // latch
    cpu.fcmd = CMD_BURST_PROG;
    cpu.latched = true;
    cpu.flashRegWrite(FSTAT, FCBEF); // launch
    try testing.expectEqual(@as(u8, 0x5A), cpu.mem[0xE000]);
    try testing.expectEqual(@as(u8, FCBEF | FCCF), cpu.fstat);
}

test "flash controller: page and mass erase set 0xFF" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.mem[0xE010] = 0x00;
    cpu.mem[0xF000] = 0x00;
    cpu.write(0xE010, 0); // latch
    cpu.fcmd = CMD_PAGE_ERASE;
    cpu.latched = true;
    cpu.flashRegWrite(FSTAT, FCBEF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xE010]);
    try testing.expectEqual(@as(u8, 0x00), cpu.mem[0xF000]); // different page untouched
    cpu.write(0xF000, 0);
    cpu.fcmd = CMD_MASS_ERASE;
    cpu.latched = true;
    cpu.flashRegWrite(FSTAT, FCBEF);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xF000]);
}

test "core: ADD/ADC carry chain and CMP#" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.mem[0x20] = 0xFF;
    cpu.mem[0x21] = 0x01;
    cpu.a = 0x01;
    cpu.add(cpu.mem[0x20], false); // 0x01 + 0xFF = 0x00, carry out
    try testing.expectEqual(@as(u8, 0x00), cpu.a);
    try testing.expect(cpu.ccr & C != 0);
    cpu.a = 0x00;
    cpu.add(cpu.mem[0x21], true); // 0x00 + 0x01 + C(1) = 0x02
    try testing.expectEqual(@as(u8, 0x02), cpu.a);
    cpu.a = 0xFF;
    cpu.cmp(0xFF);
    try testing.expect(cpu.ccr & Z != 0); // equal
    cpu.cmp(0x00);
    try testing.expect(cpu.ccr & Z == 0);
}

test "core: JSR/RTS, PSH/PUL, SP-relative and TSX/TXS round-trip" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.sp = 0x01FF;
    // 0300: LDA #$11 ; PSHA ; LDA #$22 ; STA 1,SP (9E E7 01) ; PULA ; STA $40
    //       JSR $0320 ; STA $41 ; TSX ; STHX $42 ; BGND
    // 0320: LDA #$5A ; RTS
    cpu.hostWrite(0x0320, &.{ 0xA6, 0x5A, 0x81 });
    const prog = [_]u8{
        0xA6, 0x11, 0x87, 0xA6, 0x22, 0x9E, 0xE7, 0x01, 0x86, 0xB7, 0x40,
        0xCD, 0x03, 0x20, 0xB7, 0x41, 0x95, 0x35, 0x42, 0x82,
    };
    try runProg(&cpu, 0x0300, &prog);
    try testing.expectEqual(@as(u8, 0x22), cpu.mem[0x40]); // STA 1,SP overwrote the pushed byte
    try testing.expectEqual(@as(u8, 0x5A), cpu.mem[0x41]);
    try testing.expectEqual(@as(u16, 0x01FF), cpu.sp); // balanced
    try testing.expectEqual(@as(u16, 0x0200), std.mem.readInt(u16, cpu.mem[0x42..0x44], .big)); // TSX = SP+1
}

test "core: shifts, rotates, NEG/COM/INC/DEC flags, MUL, DIV, DBNZ, CBEQ, MOV, BRSET" {
    var cpu = try testCpu();
    defer cpu.deinit();
    cpu.sp = 0x01FF;
    // LDA #$81 ; LSLA (48) -> 0x02 C=1 ; ROLA (49) -> 0x05 C=0 ; STA $50
    // LDX #7 ; LDA #6 ; MUL (42) -> X:A = 0x002A ; STA $51
    // CLRH ; LDA #100 ; LDX #7 ; DIV (52) -> A=14 H=2 ; STA $52 ; PSHH ; PULA ; STA $53
    // LDA #3 ; loop: DBNZA loop (4B FE) ; STA $54 (=0)
    // MOV #$77,$55 ; LDA #$77 ; CBEQ $55,+2 (31 55 02) ; BRA fail(20 FE) ; BSET 3,$56 ; BRSET 3,$56,+1 ; BGND ; BGND
    const prog = [_]u8{
        0xA6, 0x81, 0x48, 0x49, 0xB7, 0x50,
        0xAE, 0x07, 0xA6, 0x06, 0x42, 0xB7,
        0x51, 0x8C, 0xA6, 100,  0xAE, 0x07,
        0x52, 0xB7, 0x52, 0x8B, 0x86, 0xB7,
        0x53, 0xA6, 0x03, 0x4B, 0xFE, 0xB7,
        0x54, 0x6E, 0x77, 0x55, 0xA6, 0x77,
        0x31, 0x55, 0x02, 0x20, 0xFE, 0x16,
        0x56, 0x06, 0x56, 0x01, 0x82, 0x82,
    };
    try runProg(&cpu, 0x0300, &prog);
    try testing.expectEqual(@as(u8, 0x05), cpu.mem[0x50]);
    try testing.expectEqual(@as(u8, 42), cpu.mem[0x51]);
    try testing.expectEqual(@as(u8, 14), cpu.mem[0x52]);
    try testing.expectEqual(@as(u8, 2), cpu.mem[0x53]);
    try testing.expectEqual(@as(u8, 0), cpu.mem[0x54]);
    try testing.expectEqual(@as(u8, 0x77), cpu.mem[0x55]);
    try testing.expectEqual(@as(u8, 0x08), cpu.mem[0x56]);
    try testing.expectEqual(@as(u16, 0x0300 + prog.len), cpu.pc); // BRSET skipped the first BGND
    // SBC/borrow and signed branch: 0x10 - 0x20 = 0xF0, C=1, BLT taken (N^V=1)
    cpu.ccr = 0;
    cpu.a = 0x10;
    cpu.a = cpu.sub(cpu.a, 0x20, false);
    try testing.expectEqual(@as(u8, 0xF0), cpu.a);
    try testing.expect(cpu.flag(C) and cpu.cond(0x91));
}

fn testS08p() !Cpu {
    return Cpu.initS08p(testing.allocator, .{
        .flash = .{ .{ .lo = 0x1040, .hi = 0x2FFF }, .{ .lo = 0x3200, .hi = 0xFFFF } },
        .eeprom = .{ .lo = 0x3100, .hi = 0x31FF },
    });
}

fn p08Cmd(cpu: *Cpu, words: []const u16) void {
    cpu.hostWrite(0x3026, &.{P_ACCERR | P_FPVIOL});
    for (words, 0..) |w, i| {
        cpu.hostWrite(0x3022, &.{@intCast(i)});
        cpu.hostWrite(0x302A, &.{ @truncate(w >> 8), @truncate(w) });
    }
    cpu.hostWrite(0x3026, &.{P_CCIF});
}

test "s08p NVM: FCLKDIV gate, program/erase flash + EEPROM, erase-all unsecures" {
    var cpu = try testS08p();
    defer cpu.deinit();
    // No FCLKDIV since reset: ACCERR, nothing programmed.
    p08Cmd(&cpu, &.{ 0x0600, 0xC000, 0x1122, 0x3344 });
    try testing.expect(cpu.p.fstat & P_ACCERR != 0);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xC000]);
    cpu.hostWrite(0x3020, &.{0x07});
    var b: [1]u8 = undefined;
    cpu.hostRead(0x3020, &b);
    try testing.expectEqual(@as(u8, 0x87), b[0]); // FDIVLD
    p08Cmd(&cpu, &.{ 0x0600, 0xC000, 0x1122, 0x3344, 0x5566, 0x7788 });
    try testing.expectEqual(@as(u8, P_CCIF), cpu.p.fstat);
    try testing.expectEqualSlices(u8, &.{ 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88 }, cpu.mem[0xC000..0xC008]);
    // Misaligned program: ACCERR.
    p08Cmd(&cpu, &.{ 0x0600, 0xC102, 0, 0 });
    try testing.expect(cpu.p.fstat & P_ACCERR != 0);
    // Sector erase (512).
    p08Cmd(&cpu, &.{ 0x0A00, 0xC1FF });
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xC000]);
    // EEPROM: program 3 bytes, erase a 2-byte sector.
    p08Cmd(&cpu, &.{ 0x1100, 0x3110, 0x00AA, 0x00BB, 0x00CC });
    try testing.expectEqualSlices(u8, &.{ 0xAA, 0xBB, 0xCC }, cpu.mem[0x3110..0x3113]);
    p08Cmd(&cpu, &.{ 0x1200, 0x3110 });
    try testing.expectEqualSlices(u8, &.{ 0xFF, 0xFF, 0xCC }, cpu.mem[0x3110..0x3113]);
    // Secured after reset (NV_FSEC erased), program refused; Erase All Blocks unsecures.
    cpu.mem[0xFF7F] = 0xFF;
    cpu.resetFlashRegs();
    cpu.hostWrite(0x3020, &.{0x07});
    p08Cmd(&cpu, &.{ 0x0600, 0xC000, 0, 0 });
    try testing.expect(cpu.p.fstat & P_ACCERR != 0);
    p08Cmd(&cpu, &.{0x0800});
    try testing.expectEqual(@as(u8, P_CCIF), cpu.p.fstat);
    try testing.expectEqual(@as(u8, 0x02), cpu.p.fsec & 0x03);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0x3112]); // EEPROM erased too
    // CPU stores to flash do nothing on this controller.
    cpu.write(0xC000, 0x00);
    try testing.expectEqual(@as(u8, 0xFF), cpu.mem[0xC000]);
}
