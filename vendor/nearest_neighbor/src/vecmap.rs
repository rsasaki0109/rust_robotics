use alloc::vec::Vec;

/// Faster BTreeMap that provide only `insert` and `get`
pub struct VecMap<T> {
    vec: Vec<Option<T>>,
}

impl<T: Clone> VecMap<T> {
    pub fn new() -> Self {
        VecMap {
            vec: Vec::<Option<T>>::new(),
        }
    }

    pub fn insert(&mut self, index: usize, item: T) -> Option<T> {
        if index < self.vec.len() {
            let old = self.vec[index].clone();
            self.vec[index] = Some(item);
            return old;
        }

        while self.vec.len() <= index {
            self.vec.push(None);
        }
        self.vec[index] = Some(item);
        None
    }

    pub fn get(&self, &index: &usize) -> Option<&T> {
        if index < self.vec.len() {
            return self.vec[index].as_ref();
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_insert() {
        let mut map = VecMap::new();
        assert_eq!(map.insert(37, "a"), None);

        map.insert(37, "b");
        assert_eq!(map.insert(37, "c"), Some("b"));
    }

    #[test]
    fn test_get() {
        let mut map = VecMap::new();

        assert_eq!(map.get(&1), None);

        map.insert(1, "a");
        assert_eq!(map.get(&1), Some(&"a"));
        assert_eq!(map.get(&2), None);
    }
}
