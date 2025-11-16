use super::{ActorSnapshot, Timestamp};
use crate::rpc::ActorId;
use carla_sys::carla_rust::client::{FfiActorSnapshotList, FfiWorldSnapshot};
use cxx::UniquePtr;
use derivative::Derivative;
use static_assertions::assert_impl_all;

/// Snapshot of the world state at a specific simulation frame.
///
/// A `WorldSnapshot` captures the state of all actors in the world at a single moment in time.
/// It provides information about each actor's position, velocity, acceleration, and other state
/// without querying the simulation. This is useful for:
/// - Analyzing the state of all actors simultaneously
/// - Getting consistent data for multiple actors at the same frame
/// - Avoiding race conditions when reading actor states
///
/// Snapshots are typically obtained from [`World::wait_for_tick()`](crate::client::World::wait_for_tick).
#[cfg_attr(
    carla_version_0916,
    doc = " See [carla.WorldSnapshot](https://carla.readthedocs.io/en/0.9.16/python_api/#carla.WorldSnapshot) in the Python API."
)]
#[cfg_attr(
    carla_version_0915,
    doc = " See [carla.WorldSnapshot](https://carla.readthedocs.io/en/0.9.15/python_api/#carla.WorldSnapshot) in the Python API."
)]
#[cfg_attr(
    carla_version_0914,
    doc = " See [carla.WorldSnapshot](https://carla.readthedocs.io/en/0.9.14/python_api/#carla.WorldSnapshot) in the Python API."
)]
///
/// # Examples
///
/// ```no_run
/// use carla::client::Client;
///
/// let client = Client::default();
/// let world = client.world();
///
/// // Wait for next tick and get snapshot
/// let snapshot = world.wait_for_tick();
///
/// println!(
///     "Frame: {}, Time: {:.2}s",
///     snapshot.frame(),
///     snapshot.timestamp().elapsed_seconds
/// );
///
/// // Check all actor velocities
/// for actor_snapshot in snapshot.actor_snapshots() {
///     let speed = actor_snapshot.velocity().norm();
///     if speed > 0.1 {
///         println!("Actor {} moving at {:.1} m/s", actor_snapshot.id(), speed);
///     }
/// }
/// ```
#[derive(Derivative)]
#[derivative(Debug)]
#[repr(transparent)]
pub struct WorldSnapshot {
    #[derivative(Debug = "ignore")]
    inner: UniquePtr<FfiWorldSnapshot>,
}

impl WorldSnapshot {
    /// Returns the unique identifier for this snapshot.
    ///
    /// Each snapshot has a unique ID that increments with each simulation tick.
    pub fn id(&self) -> u64 {
        self.inner.GetId()
    }

    /// Returns the simulation frame number.
    ///
    /// The frame number starts at 0 and increments by 1 for each simulation tick.
    /// This is useful for tracking simulation progress and synchronizing data.
    pub fn frame(&self) -> usize {
        self.inner.GetFrame()
    }

    /// Returns the timestamp information for this snapshot.
    ///
    /// The timestamp contains:
    /// - Frame number
    /// - Elapsed simulation time
    /// - Delta time since last tick
    /// - Platform timestamp
    ///
    /// See [`Timestamp`] for more details.
    pub fn timestamp(&self) -> &Timestamp {
        self.inner.GetTimestamp()
    }

    /// Checks if an actor exists in this snapshot.
    ///
    /// Returns `true` if the actor was present in the simulation at the time
    /// of this snapshot, `false` otherwise.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use carla::client::Client;
    /// # let client = Client::default();
    /// # let world = client.world();
    /// let snapshot = world.wait_for_tick();
    /// let actor_id = 42;
    ///
    /// if snapshot.contains(actor_id) {
    ///     println!("Actor {} exists", actor_id);
    /// }
    /// ```
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
