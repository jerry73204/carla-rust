pub use carla_sys::carla::road::{
    JuncId, LaneId, Lane_LaneType as LaneType, RoadId, SectionId, SignalOrientation,
};
pub type SignId = String;

pub mod element {
    use carla_sys::carla_rust::road::element::FfiLaneMarking;
    use cxx::UniquePtr;

    pub use carla_sys::carla::road::element::{
        LaneMarking_Color, LaneMarking_LaneChange, LaneMarking_Type,
    };
    use derivative::Derivative;
    use static_assertions::assert_impl_all;

    #[derive(Derivative)]
    #[derivative(Debug)]
    pub struct LaneMarking {
        #[derivative(Debug = "ignore")]
        inner: UniquePtr<FfiLaneMarking>,
    }

    impl LaneMarking {
        pub fn type_(&self) -> LaneMarking_Type {
            self.inner.type_()
        }

        pub fn color(&self) -> LaneMarking_Color {
            self.inner.color()
        }

        pub fn lane_change(&self) -> LaneMarking_LaneChange {
            self.inner.lane_change()
        }

        pub fn width(&self) -> f64 {
            self.inner.width()
        }

        pub(crate) fn from_cxx(ptr: UniquePtr<FfiLaneMarking>) -> Option<Self> {
            if ptr.is_null() {
                None
            } else {
                Some(Self { inner: ptr })
            }
        }
    }

    assert_impl_all!(LaneMarking: Send);
}
