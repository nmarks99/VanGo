
// linear mapping between two ranges, similar to arduino's map function
pub fn map(x:u32, xmin:u32, xmax:u32, ymin:u32, ymax:u32) -> u32 {
    x*(ymax-ymin) / (xmax - xmin) + ymin
}
