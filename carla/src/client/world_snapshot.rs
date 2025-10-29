use super::{ActorSnapshot, Timestamp};
use crate::rpc::ActorId;
use carla_sys::carla_rust::client::{FfiActorSnapshotList, FfiWorldSnapshot};
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Provides information for every actor at a certain moment of time,
/// Corresponds to [`carla.WorldSnapshot`](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WorldSnapshot) in the Python API.
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct WorldSnapshot {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiWorldSnapshot>,
}

impl WorldSnapshot {
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    pub fn frame(&self) -> usize {
        self.inner.GetFrame()
    }

    pub fn timestamp(&self) -> &Timestamp {
        self.inner.GetTimestamp()
    }

    pub fn contains(&self, actor_id: ActorId) -> bool {
        self.inner.Contains(actor_id)
    }

    /// Find an actor snapshot by actor ID.
    ///
    /// Returns `None` if the actor is not present in this snapshot (either it doesn't exist
    /// or hasn't been spawned yet at this frame).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let snapshot = world.wait_for_tick();
    ///
    /// // Check specific actor
    /// let actor_id = 42;
    /// if let Some(actor_snapshot) = snapshot.find(actor_id) {
    ///     let velocity = actor_snapshot.velocity();
    ///     println!("Actor {} velocity: {:?}", actor_id, velocity);
    /// }
    /// ```
    pub fn find(&self, actor_id: ActorId) -> Option<ActorSnapshot> {
        ActorSnapshot::from_cxx(self.inner.Find(actor_id))
    }

    /// Returns an iterator over all actor snapshots in this world snapshot.
    ///
    /// This is more efficient than querying actors individually, especially when examining
    /// many actors' states simultaneously.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use carla::client::Client;
    ///
    /// let client = Client::default();
    /// let world = client.world();
    /// let snapshot = world.wait_for_tick();
    ///
    /// // Find all fast-moving actors
    /// for actor_snapshot in snapshot.actor_snapshots() {
    ///     let speed = actor_snapshot.velocity().norm();
    ///     if speed > 20.0 {
    ///         println!(
    ///             "Actor {} is moving at {:.1} m/s",
    ///             actor_snapshot.id(),
    ///             speed
    ///         );
    ///     }
    /// }
    /// ```
    pub fn actor_snapshots(&self) -> ActorSnapshotIter {
        ActorSnapshotIter {
            list: self.inner.GetActorSnapshots(),
            index: 0,
        }
    }

    pub(crate) fn from_cxx(ptr: UniquePtr<FfiWorldSnapshot>) -> Option<Self> {
        if ptr.is_null() {
            None
        } else {
            Some(Self { inner: ptr })
        }
    }
}

/// Iterator over actor snapshots in a world snapshot.
///
/// Created by [`WorldSnapshot::actor_snapshots()`].
pub struct ActorSnapshotIter {
    list: UniquePtr<FfiActorSnapshotList>,
    index: usize,
}

impl Iterator for ActorSnapshotIter {
    type Item = ActorSnapshot;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.list.size() {
            return None;
        }

        let snapshot = ActorSnapshot::from_cxx(self.list.get(self.index))?;
        self.index += 1;
        Some(snapshot)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.list.size().saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for ActorSnapshotIter {
    fn len(&self) -> usize {
        self.list.size().saturating_sub(self.index)
    }
}

assert_impl_all!(WorldSnapshot: Send);
