use core::cmp;

pub fn clamp<T: cmp::Ord>(min: T, value: T, max: T) -> T {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}