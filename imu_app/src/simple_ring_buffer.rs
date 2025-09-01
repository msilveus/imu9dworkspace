#![allow(unused_variables)]

/// A fixed-capacity ring buffer for Copy types.
/// Works well as a delay line in embedded apps.
pub struct RingBuffer<T: Copy, const N: usize> {
    buffer: [T; N],
    head: usize,
    tail: usize,
    size: usize,
}

impl<T: Copy + Default, const N: usize> RingBuffer<T, N> {
    /// Create a new empty buffer
    pub fn new() -> Self {
        Self {
            buffer: [T::default(); N],
            head: 0,
            tail: 0,
            size: 0,
        }
    }
    
    pub const fn new_filled(fill: T) -> Self {
        Self {
            buffer: [fill; N],
            head: 0,
            tail: 0,
            size: 0,
        }
    }

    pub fn reset(&mut self) {
        self.head = 0;
        self.tail = 0;
        self.size = 0;
    }

    pub fn count(&self) -> usize {
        self.size
    }

    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    pub fn is_full(&self) -> bool {
        self.size == N
    }

    /// Push without overwrite (fails if full)
    pub fn push(&mut self, val: T) -> bool {
        if self.is_full() {
            return false;
        }
        self.buffer[self.tail] = val;
        self.tail = (self.tail + 1) % N;
        self.size += 1;
        true
    }

    /// Push with overwrite (cyclic behavior)
    pub fn push_cyclic(&mut self, val: T) {
        if self.is_full() {
            // overwrite oldest
            self.head = (self.head + 1) % N;
        } else {
            self.size += 1;
        }
        self.buffer[self.tail] = val;
        self.tail = (self.tail + 1) % N;
    }

    /// Pop oldest value
    pub fn pop(&mut self) -> Option<T> {
        if self.is_empty() {
            return None;
        }
        let val = self.buffer[self.head];
        self.head = (self.head + 1) % N;
        self.size -= 1;
        Some(val)
    }

    /// Peek by index relative to head
    pub fn peek(&self, index: usize) -> Option<T> {
        if index >= self.size {
            return None;
        }
        let pos = (self.head + index) % N;
        Some(self.buffer[pos])
    }
}
