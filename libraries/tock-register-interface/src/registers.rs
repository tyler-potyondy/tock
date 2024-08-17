// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Implementation of included register types.
//!
//! This module provides a standard set of register types, which can
//! describe different access levels:
//!
//! - [`ReadWrite`] for registers which can be read and written to
//! - [`ReadOnly`] for registers which can only be read
//! - [`WriteOnly`] for registers which can only be written to
//! - [`Aliased`] for registers which can be both read and written,
//!   but represent different registers depending on the operation
//! - [`InMemoryRegister`] provide a register-type in RAM using
//!   volatile operations
//!
//! These types can be disabled by removing the `register_types` crate
//! feature (part of the default features). This is useful if this
//! crate should be used only as an interface library, or if all
//! unsafe code should be disabled.

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::mem::transmute;

use crate::fields::FieldValue;
use crate::interfaces::{Readable, Writeable};
use crate::{PersPower, PowerControl, PowerOff, PowerOn, RegisterLongName, UIntLike};

/// Read/Write registers.
///
/// For accessing and manipulating the register contents, the
/// [`Readable`], [`Writeable`] and
/// [`ReadWriteable`](crate::interfaces::ReadWriteable) traits are
/// implemented.
// To successfully alias this structure onto hardware registers in memory, this
// struct must be exactly the size of the `T` and is thus marked
// `repr(transparent)` over an `UnsafeCell<T>`, which itself is
// `repr(transparent)` over `T`.
//
// This struct is constructed by casting a pointer to it (or, implicitly, by
// casting a pointer to a larger struct that containts this type). As such, it
// does not have a public constructor and Rust thinks it's dead code and should
// be removed. We `allow(dead_code)` here to suppress this warning.
#[allow(dead_code)]
#[repr(transparent)]
pub struct ReadWrite<T: UIntLike, P: PowerControl<P>, R: RegisterLongName = ()> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<R>,
    associated_power: PhantomData<P>,
}
impl<T: UIntLike, R: RegisterLongName, P: PowerControl<P>> Readable for ReadWrite<T, P, R> {
    type T = T;
    type R = R;

    #[inline]
    fn get(&self) -> Self::T {
        unsafe { ::core::ptr::read_volatile(self.value.get()) }
    }
}
impl<T: UIntLike, R: RegisterLongName, P: PowerControl<P>> Writeable for ReadWrite<T, P, R> {
    type T = T;
    type R = R;
    type P = P;

    #[inline]
    fn set(&self, value: T, power: PowerOn<P>) -> PowerOn<P> {
        unsafe { ::core::ptr::write_volatile(self.value.get(), value) }
        power
    }
}

/// Read-only registers.
///
/// For accessing the register contents the [`Readable`] trait is
/// implemented.
// To successfully alias this structure onto hardware registers in memory, this
// struct must be exactly the size of the `T` and is thus marked
// `repr(transparent)` over an `UnsafeCell<T>`, which itself is
// `repr(transparent)` over `T`.
//
// This struct is constructed by casting a pointer to it (or, implicitly, by
// casting a pointer to a larger struct that containts this type). As such, it
// does not have a public constructor and Rust thinks it's dead code and should
// be removed. We `allow(dead_code)` here to suppress this warning.
#[allow(dead_code)]
#[repr(transparent)]
pub struct ReadOnly<T: UIntLike, R: RegisterLongName = ()> {
    value: T,
    associated_register: PhantomData<R>,
}
impl<T: UIntLike, R: RegisterLongName> Readable for ReadOnly<T, R> {
    type T = T;
    type R = R;

    #[inline]
    fn get(&self) -> T {
        unsafe { ::core::ptr::read_volatile(&self.value) }
    }
}

/// Power On Register
#[allow(dead_code)]
#[repr(transparent)]
pub struct PowerWrite<T: UIntLike, P: PowerControl<P>, R: RegisterLongName = ()> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<R>,
    associated_power: PhantomData<P>,
}

impl<T: UIntLike, P: PowerControl<P>, R: RegisterLongName> PowerWrite<T, P, R> {
    pub fn power_on(&self, power: PowerOff<P>, field: FieldValue<T, R>) -> PowerOn<P> {
        unsafe {
            ::core::ptr::write_volatile(self.value.get(), field.value);
            transmute::<PowerOff<P>, PowerOn<P>>(power)
        }
    }

    pub fn power_off(&self, power: PowerOn<P>, field: FieldValue<T, R>) -> PowerOff<P> {
        unsafe {
            // should we do this? @pat?
            ::core::ptr::write_volatile(self.value.get(), field.value);
            transmute::<PowerOn<P>, PowerOff<P>>(power)
        }
    }
}

/// PrePowerConfig Register
#[allow(dead_code)]
#[repr(transparent)]
pub struct PrePowerConfig<T: UIntLike, P: PowerControl<P>, R: RegisterLongName = ()> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<R>,
    associated_power: PhantomData<P>,
}

impl<T: UIntLike, P: PowerControl<P>, R: RegisterLongName> PrePowerConfig<T, P, R> {
    pub fn config_write(&self, power: PowerOff<P>, field: FieldValue<T, R>) -> PowerOff<P> {
        unsafe {
            ::core::ptr::write_volatile(self.value.get(), field.value);
        }
        power
    }
}

#[allow(dead_code)]
#[repr(transparent)]
pub struct PersistentPower<T: UIntLike, P: PowerControl<P>, R: RegisterLongName = ()> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<R>,
    associated_power: PhantomData<P>,
}

impl<T: UIntLike, P: PowerControl<P>, R: RegisterLongName> PersistentPower<T, P, R> {
    pub fn write(&self, field: FieldValue<T, R>, power: PowerOn<P>) -> PersPower<P> {
        unsafe {
            ::core::ptr::write_volatile(self.value.get(), field.value);
            transmute::<PowerOn<P>, PersPower<P>>(power)
        }
    }
}

/// General Register

/// Power Off Register

/// Write-only registers.
///
/// For setting the register contents the [`Writeable`] trait is
/// implemented.
// To successfully alias this structure onto hardware registers in memory, this
// struct must be exactly the size of the `T` and is thus marked
// `repr(transparent)` over an `UnsafeCell<T>`, which itself is
// `repr(transparent)` over `T`.
//
// This struct is constructed by casting a pointer to it (or, implicitly, by
// casting a pointer to a larger struct that containts this type). As such, it
// does not have a public constructor and Rust thinks it's dead code and should
// be removed. We `allow(dead_code)` here to suppress this warning.
#[allow(dead_code)]
#[repr(transparent)]
pub struct WriteOnly<T: UIntLike, P: PowerControl<P>, R: RegisterLongName = ()> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<R>,
    associated_power: PhantomData<P>,
}
impl<T: UIntLike, R: RegisterLongName, P: PowerControl<P>> Writeable for WriteOnly<T, P, R> {
    type T = T;
    type R = R;
    type P = P;

    #[inline]
    fn set(&self, value: T, power: PowerOn<P>) -> PowerOn<P> {
        unsafe { ::core::ptr::write_volatile(self.value.get(), value) }
        power
    }
}

/// Read-only and write-only registers aliased to the same address.
///
/// Unlike the [`ReadWrite`] register, this represents a register
/// which has different meanings based on if it is written or read.
/// This might be found on a device where control and status registers
/// are accessed via the same memory address via writes and reads,
/// respectively.
///
/// This register implements [`Readable`] and [`Writeable`], but in
/// general does not implement
/// [`ReadWriteable`](crate::interfaces::ReadWriteable) (only if the
/// type parameters `R` and `W` are identical, in which case a
/// [`ReadWrite`] register might be a better choice).
// To successfully alias this structure onto hardware registers in memory, this
// struct must be exactly the size of the `T` and is thus marked
// `repr(transparent)` over an `UnsafeCell<T>`, which itself is
// `repr(transparent)` over `T`.
//
// This struct is constructed by casting a pointer to it (or, implicitly, by
// casting a pointer to a larger struct that containts this type). As such, it
// does not have a public constructor and Rust thinks it's dead code and should
// be removed. We `allow(dead_code)` here to suppress this warning.
#[allow(dead_code)]
#[repr(transparent)]
pub struct Aliased<
    T: UIntLike,
    P: PowerControl<P>,
    R: RegisterLongName = (),
    W: RegisterLongName = (),
> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<(R, W)>,
    associated_power: PhantomData<P>,
}
impl<T: UIntLike, P: PowerControl<P>, R: RegisterLongName, W: RegisterLongName> Readable
    for Aliased<T, P, R, W>
{
    type T = T;
    type R = R;

    #[inline]
    fn get(&self) -> Self::T {
        unsafe { ::core::ptr::read_volatile(self.value.get()) }
    }
}
impl<T: UIntLike, R: RegisterLongName, W: RegisterLongName, P: PowerControl<P>> Writeable
    for Aliased<T, P, R, W>
{
    type T = T;
    type R = W;
    type P = P;

    #[inline]
    fn set(&self, value: Self::T, power: PowerOn<P>) -> PowerOn<P> {
        unsafe { ::core::ptr::write_volatile(self.value.get(), value) }
        power
    }
}

/// In memory volatile register.
///
/// Like [`ReadWrite`], but can be safely constructed using the
/// [`InMemoryRegister::new`] method. It will always be initialized to
/// the passed in, well-defined initial value.
///
/// For accessing and manipulating the register contents, the
/// [`Readable`], [`Writeable`] and
/// [`ReadWriteable`](crate::interfaces::ReadWriteable) traits are
/// implemented.
// To successfully alias this structure onto hardware registers in memory, this
// struct must be exactly the size of the `T`.
#[repr(transparent)]
pub struct InMemoryRegister<T: UIntLike, P: PowerControl<P>, R: RegisterLongName = ()> {
    value: UnsafeCell<T>,
    associated_register: PhantomData<R>,
    associated_power: PhantomData<P>,
}

impl<T: UIntLike, R: RegisterLongName, P: PowerControl<P>> InMemoryRegister<T, P, R> {
    pub const fn new(value: T) -> Self {
        InMemoryRegister {
            value: UnsafeCell::new(value),
            associated_register: PhantomData,
            associated_power: PhantomData,
        }
    }
}
impl<T: UIntLike, R: RegisterLongName, P: PowerControl<P>> Readable for InMemoryRegister<T, P, R> {
    type T = T;
    type R = R;

    #[inline]
    fn get(&self) -> Self::T {
        unsafe { ::core::ptr::read_volatile(self.value.get()) }
    }
}
impl<T: UIntLike, P: PowerControl<P>, R: RegisterLongName> Writeable for InMemoryRegister<T, P, R> {
    type T = T;
    type R = R;
    type P = P;

    #[inline]
    fn set(&self, value: T, power: PowerOn<P>) -> PowerOn<P> {
        unsafe { ::core::ptr::write_volatile(self.value.get(), value) }
        power
    }
}
