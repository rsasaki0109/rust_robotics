use std::mem::{size_of, MaybeUninit};
use std::ptr::null_mut;

use libc::{
    pthread_cond_broadcast,
    pthread_cond_init,
    pthread_cond_signal,
    //Events
    pthread_cond_t,
    pthread_cond_timedwait,
    pthread_cond_wait,
    pthread_condattr_init,
    pthread_condattr_setpshared,

    pthread_condattr_t,
    PTHREAD_PROCESS_SHARED,
};
//use log::*;

use crate::events::*;
use crate::locks::*;
use crate::{Result, Timeout};

struct InnerEvent {
    cond: pthread_cond_t,
    auto_reset: u8,
    signal: u8,
}
pub struct Event {
    mutex: Box<dyn LockImpl>,
    inner: *mut InnerEvent,
}
impl EventInit for Event {
    fn size_of(addr: Option<*mut u8>) -> usize {
        let mutex_size = Mutex::size_of(addr);
        let padding = match addr {
            Some(mem) => unsafe { mem.add(mutex_size).align_offset(size_of::<*mut u8>() as _) },
            None => 0,
        };
        Mutex::size_of(addr) + padding + size_of::<InnerEvent>()
    }

    #[allow(clippy::new_ret_no_self)]
    unsafe fn new(mem: *mut u8, auto_reset: bool) -> Result<(Box<dyn EventImpl>, usize)> {
        let (mutex, used_bytes) = Mutex::new(mem, null_mut())?;
        let ptr = mem.add(used_bytes);
        let ptr = ptr.add(ptr.align_offset(size_of::<*mut u8>() as _)) as *mut InnerEvent;
        let inner = &mut *ptr;

        #[allow(clippy::uninit_assumed_init)]
        let mut attrs: pthread_condattr_t = MaybeUninit::uninit().assume_init();
        //trace!("pthread_condattr_init()");
        if pthread_condattr_init(&mut attrs) != 0 {
            return Err(From::from(
                "Failed to initialize pthread_condattr_init".to_string(),
            ));
        }
        //trace!("pthread_condattr_setpshared()");
        if pthread_condattr_setpshared(&mut attrs, PTHREAD_PROCESS_SHARED) != 0 {
            return Err(From::from(
                "Failed to set pthread_condattr_setpshared(PTHREAD_PROCESS_SHARED)".to_string(),
            ));
        }

        //trace!("pthread_cond_init({:p})", ptr);
        if pthread_cond_init(&mut inner.cond, &attrs) != 0 {
            return Err(From::from(
                "Failed to initialize pthread_cond_init".to_string(),
            ));
        }
        inner.auto_reset = if auto_reset { 1 } else { 0 };
        inner.signal = 0;

        let obj = Box::new(Self { mutex, inner });

        Ok((obj, (ptr as usize - mem as usize) + Self::size_of(None)))
    }

    unsafe fn from_existing(mem: *mut u8) -> Result<(Box<dyn EventImpl>, usize)> {
        let (mutex, used_bytes) = Mutex::from_existing(mem, null_mut())?;
        let ptr = mem.add(used_bytes);
        let ptr = ptr.add(ptr.align_offset(size_of::<*mut u8>() as _)) as *mut InnerEvent;

        let inner = &mut *ptr;

        if inner.auto_reset > 1 || inner.signal > 1 {
            return Err(From::from("Existing Event is corrupted"));
        }

        let obj = Box::new(Self { mutex, inner });

        Ok((obj, (ptr as usize - mem as usize) + Self::size_of(None)))
    }
}

impl EventImpl for Event {
    fn wait(&self, timeout: Timeout) -> Result<()> {
        let (guard, timespec) = match timeout {
            Timeout::Infinite => (self.mutex.lock()?, None),
            Timeout::Val(d) => {
                let timespec = abs_timespec_from_duration(d);
                (self.mutex.try_lock(timeout)?, Some(timespec))
            }
        };

        let inner = unsafe { &mut *self.inner };
        let mut res = 0;
        if let Some(ts) = timespec {
            while inner.signal != 1 {
                res = unsafe {
                    pthread_cond_timedwait(&mut inner.cond, self.mutex.as_raw() as _, &ts)
                };
                if res != 0 {
                    break;
                }
            }
        } else {
            while inner.signal != 1 {
                res = unsafe { pthread_cond_wait(&mut inner.cond, self.mutex.as_raw() as _) };
                if res != 0 {
                    break;
                }
            }
        }

        // Success
        let ret = if res == 0 {
            if inner.auto_reset == 1 {
                inner.signal = 0;
            }
            Ok(())
        } else {
            Err(From::from("Failed waiting for signal".to_string()))
        };

        drop(guard);
        ret
    }

    fn set(&self, state: EventState) -> Result<()> {
        let guard = self.mutex.lock()?;
        let inner = unsafe { &mut *self.inner };
        let res = match state {
            EventState::Clear => {
                //trace!("reset pthread_cond({:p})", &inner.cond);
                inner.signal = 0;
                0
            }
            EventState::Signaled => {
                inner.signal = 1;
                unsafe {
                    if inner.auto_reset == 1 {
                        //trace!("pthread_cond_signal({:p})", &inner.cond);
                        pthread_cond_signal(&mut inner.cond)
                    } else {
                        //trace!("pthread_cond_broadcast({:p})", &inner.cond);
                        pthread_cond_broadcast(&mut inner.cond)
                    }
                }
            }
        };
        drop(guard);

        if res != 0 {
            Err(From::from(format!(
                "Failed to set event state : 0x{:X}",
                res
            )))
        } else {
            Ok(())
        }
    }
}
