use std::cell::UnsafeCell;
use std::ffi::CString;
use std::mem::size_of;
use std::ptr::null_mut;

pub const MUTEX_ALL_ACCESS: u32 = 0x1F0001;
use winapi::{
    shared::ntdef::{FALSE, NULL},
    um::{
        handleapi::CloseHandle,
        synchapi::{CreateMutexExA, ReleaseMutex, WaitForSingleObject, CREATE_MUTEX_INITIAL_OWNER},
        winbase::{OpenMutexA, INFINITE, WAIT_ABANDONED, WAIT_OBJECT_0},
        winnt::{HANDLE, SYNCHRONIZE},
    },
};

use super::{LockGuard, LockImpl, LockInit};
use crate::{Result, Timeout};

pub struct Mutex {
    handle: HANDLE,
    data: UnsafeCell<*mut u8>,
}

impl LockInit for Mutex {
    fn size_of(_addr: Option<*mut u8>) -> usize {
        size_of::<u32>()
    }

    #[allow(clippy::new_ret_no_self)]
    unsafe fn new(mem: *mut u8, data: *mut u8) -> Result<(Box<dyn LockImpl>, usize)> {
        // Find a mutex id that doesnt collide with another
        let mut mutex_handle: HANDLE = NULL;
        let mut mutex_id: u32 = 0;
        while mutex_handle == NULL {
            mutex_id = rand::random::<u32>();
            let path = CString::new(format!("mutex_{}", mutex_id)).unwrap();
            //trace!("CreateMutexExA(NULL, '{}', 0x{:X}, 0x{:X})",path.to_string_lossy(),CREATE_MUTEX_INITIAL_OWNER,MUTEX_ALL_ACCESS);
            mutex_handle = CreateMutexExA(
                null_mut(),
                path.as_ptr() as *mut _,
                CREATE_MUTEX_INITIAL_OWNER,
                MUTEX_ALL_ACCESS,
            );
        }

        // Create our mutex struct
        let mutex = Box::new(Self {
            handle: mutex_handle,
            data: UnsafeCell::new(data),
        });
        mutex.release()?;

        // Write the mutex id to the backing memory
        *(mem as *mut u32) = mutex_id;

        Ok((mutex, Self::size_of(None)))
    }

    unsafe fn from_existing(mem: *mut u8, data: *mut u8) -> Result<(Box<dyn LockImpl>, usize)> {
        let mutex_id = *(mem as *mut u32);
        let path = CString::new(format!("mutex_{}", mutex_id)).unwrap();
        //trace!("OpenMutexA(0x{:X}, 0x{:X}, '{}')", SYNCHRONIZE,FALSE,path.to_string_lossy());
        let mutex_handle = OpenMutexA(SYNCHRONIZE, FALSE as _, path.as_ptr() as *mut _);
        if mutex_handle == NULL {
            return Err(From::from(format!(
                "Failed to open mutex {}",
                path.to_string_lossy()
            )));
        }

        let mutex = Box::new(Self {
            handle: mutex_handle,
            data: UnsafeCell::new(data),
        });

        Ok((mutex, Self::size_of(None)))
    }
}

impl Drop for Mutex {
    fn drop(&mut self) {
        //trace!("CloseHandle(0x{:X})", self.handle as usize);
        unsafe { CloseHandle(self.handle) };
    }
}

impl LockImpl for Mutex {
    fn as_raw(&self) -> *mut std::ffi::c_void {
        self.handle as _
    }

    fn lock(&self) -> Result<LockGuard<'_>> {
        let wait_res = unsafe { WaitForSingleObject(self.handle, INFINITE) };
        //trace!("WaitForSingleObject(0x{:X})", self.handle as usize);
        if wait_res == WAIT_OBJECT_0 {
            Ok(LockGuard::new(self))
        } else if wait_res == WAIT_ABANDONED {
            panic!("A thread holding the mutex has left it in a poisened state");
        } else {
            Err(From::from(format!(
                "Failed to aquire lock with value : 0x{:X}",
                wait_res
            )))
        }
    }

    fn try_lock(&self, timeout: Timeout) -> Result<LockGuard<'_>> {
        let wait_res = unsafe {
            WaitForSingleObject(
                self.handle,
                match timeout {
                    Timeout::Infinite => INFINITE,
                    Timeout::Val(d) => d.as_millis() as u32,
                },
            )
        };
        //trace!("WaitForSingleObject(0x{:X})", self.handle as usize);
        if wait_res == WAIT_OBJECT_0 {
            Ok(LockGuard::new(self))
        } else if wait_res == WAIT_ABANDONED {
            panic!("A thread holding the mutex has left it in a poisened state");
        } else {
            Err(From::from(format!(
                "Failed to aquire lock with value : 0x{:X}",
                wait_res
            )))
        }
    }

    fn release(&self) -> Result<()> {
        //trace!("ReleaseMutex(0x{:X})", self.handle as usize);
        if unsafe { ReleaseMutex(self.handle) } == 0 {
            Err(From::from(
                "Could not release mutex as we did not own it".to_string(),
            ))
        } else {
            Ok(())
        }
    }
    unsafe fn get_inner(&self) -> &mut *mut u8 {
        &mut *self.data.get()
    }
}
