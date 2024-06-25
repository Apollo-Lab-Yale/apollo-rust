use crate::isometry3::I3;
use crate::quaternions::Q;
use crate::vectors::V3;

pub type ISE3q = LieGroupISE3q;
pub type Ise3q = LieAlgISE3q;

pub struct LieGroupISE3q(pub I3);
impl LieGroupISE3q {
    pub fn new(field0: I3) -> Self {
        Self(field0)
    }
}

pub struct LieAlgISE3q { q: Q, t: V3 }
impl LieAlgISE3q {
    pub fn new(q: Q, t: V3) -> Self {
        Self { q, t }
    }
    #[inline(always)]
    pub fn q(&self) -> &Q {
        &self.q
    }
    #[inline(always)]
    pub fn t(&self) -> &V3 {
        &self.t
    }
}