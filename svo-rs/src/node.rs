use crate::{Error, Vector3};

use hashbrown::HashMap;

use alloc::{boxed::Box, vec::Vec};
use core::{
    convert::TryFrom,
    fmt::Debug,
    hash::Hash,
    ops::{Deref, DerefMut},
};

const BOUNDS_LEN: usize = 2;

pub const OCTREE_CHILDREN: usize = 8;

pub type Bounds = [Vector3<u32>; BOUNDS_LEN];

#[repr(usize)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Octant {
    LeftRearBase = 0,
    RightRearBase = 1,
    LeftRearTop = 2,
    RightRearTop = 3,
    LeftFrontBase = 4,
    RightFrontBase = 5,
    LeftFrontTop = 6,
    RightFrontTop = 7,
}

impl TryFrom<usize> for Octant {
    type Error = Error;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::LeftRearBase),
            1 => Ok(Self::RightRearBase),
            2 => Ok(Self::LeftRearTop),
            3 => Ok(Self::RightRearTop),
            4 => Ok(Self::LeftFrontBase),
            5 => Ok(Self::RightFrontBase),
            6 => Ok(Self::LeftFrontTop),
            7 => Ok(Self::RightFrontTop),
            _ => Err(Error::InvalidOctant(value)),
        }
    }
}

impl Octant {
    fn offset(&self) -> Vector3<u32> {
        match self {
            Self::LeftRearBase => Vector3::from([0, 0, 0]),
            Self::RightRearBase => Vector3::from([1, 0, 0]),
            Self::LeftRearTop => Vector3::from([0, 0, 1]),
            Self::RightRearTop => Vector3::from([1, 0, 1]),
            Self::LeftFrontBase => Vector3::from([0, 1, 0]),
            Self::RightFrontBase => Vector3::from([1, 1, 0]),
            Self::LeftFrontTop => Vector3::from([0, 1, 1]),
            Self::RightFrontTop => Vector3::from([1, 1, 1]),
        }
    }

    fn vector_diff(rhs: Vector3<u32>, lhs: Vector3<u32>) -> Self {
        if lhs.z < rhs.z {
            if lhs.y < rhs.y {
                if lhs.x < rhs.x {
                    Self::LeftRearBase
                } else {
                    Self::RightRearBase
                }
            } else {
                if lhs.x < rhs.x {
                    Self::LeftFrontBase
                } else {
                    Self::RightFrontBase
                }
            }
        } else {
            if lhs.y < rhs.y {
                if lhs.x < rhs.x {
                    Self::LeftRearTop
                } else {
                    Self::RightRearTop
                }
            } else {
                if lhs.x < rhs.x {
                    Self::LeftFrontTop
                } else {
                    Self::RightFrontTop
                }
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NodeType<T> {
    Leaf(T),
    Internal,
}

impl<T> Default for NodeType<T> {
    fn default() -> Self {
        Self::Internal
    }
}

struct ChildInfo {
    dimension: u32,
    dimension_3d: Vector3<u32>,
    octant: Octant,
}

#[derive(Debug, Default, Clone)]
pub struct Node<T>
where
    T: Debug + Default + Eq + PartialEq + Clone + Copy + Hash,
{
    pub ty: NodeType<T>,
    pub bounds: Bounds,
    pub children: [Box<Option<Node<T>>>; OCTREE_CHILDREN],
}

impl<T> Node<T>
where
    T: Debug + Default + Eq + PartialEq + Clone + Copy + Hash,
{
    /// Creates a new `Node<T>` with the given bounds.
    pub(crate) fn new(bounds: Bounds) -> Self {
        Self {
            ty: NodeType::Leaf(Default::default()),
            bounds,
            ..Default::default()
        }
    }

    /// Inserts a new leaf `Node` at the given position, if possible.
    pub(crate) fn insert(&mut self, position: Vector3<u32>, min_dimension: u32, data: T) -> Result<(), Error> {
        if !self.contains(position) {
            return Err(Error::InvalidPosition {
                x: position.x,
                y: position.y,
                z: position.z,
            });
        }

        if let NodeType::Leaf(cur) = self.ty {
            if cur == data {
                return Ok(());
            }
        }

        if self.dimension() == min_dimension {
            self.ty = NodeType::Leaf(data);
        } else {
            let ChildInfo {
                dimension,
                dimension_3d,
                octant,
            } = self.child_info(position).unwrap();

            let bounds = self.child_bounds(dimension_3d, octant);

            let mut node = if self.children[octant as usize].as_ref().is_some() {
                self.children[octant as usize].take().unwrap()
            } else {
                Node::<T>::new(bounds)
            };

            if self.is_leaf() && dimension == min_dimension {
                for i in 0..OCTREE_CHILDREN {
                    if i != octant as usize {
                        let new_octant = Octant::try_from(i).unwrap();
                        let bounds = self.child_bounds(dimension_3d, new_octant);

                        let mut new_node = Node::<T>::new(bounds);
                        new_node.ty = NodeType::Leaf(*self.leaf_data().unwrap());

                        self.children[new_octant as usize] = Box::new(Some(new_node));
                    }
                }
            }

            node.insert(position, min_dimension, data).unwrap();

            self.children[octant as usize] = Box::new(Some(node));
            self.ty = NodeType::Internal;

            self.simplify();
        }

        Ok(())
    }

    /// Removes the `Node` at the given position, if possible.
    pub(crate) fn clear(&mut self, position: Vector3<u32>, min_dimension: u32) -> Result<(), Error> {
        if self.contains(position) {
            let ChildInfo {
                dimension,
                dimension_3d,
                octant,
            } = self.child_info(position).unwrap();

            if self.is_leaf() && dimension == min_dimension {
                for i in 0..OCTREE_CHILDREN {
                    let (octant, data) = if i != octant as usize {
                        (Octant::try_from(i).unwrap(), *self.leaf_data().unwrap())
                    } else {
                        (octant, Default::default())
                    };

                    let bounds = self.child_bounds(dimension_3d, octant);
                    let mut node = Node::<T>::new(bounds);
                    node.ty = NodeType::Leaf(data);

                    self.children[i].deref_mut().replace(node);
                }
            } else if self.children[octant as usize].as_ref().is_some() {
                let mut child = self.children[octant as usize].take().unwrap();
                child.clear(position, min_dimension).unwrap();

                child.ty = if self.is_leaf() || dimension == min_dimension {
                    NodeType::Leaf(Default::default())
                } else {
                    NodeType::Internal
                };

                self.children[octant as usize].deref_mut().replace(child);
            }

            Ok(())
        } else {
            Err(Error::InvalidPosition {
                x: position.x,
                y: position.y,
                z: position.z,
            })
        }
    }

    /// Gets data from a `Node` at the given position, if possible.
    pub(crate) fn get(&self, position: Vector3<u32>) -> Option<&T> {
        if self.contains(position) {
            return match &self.ty {
                NodeType::Leaf(data) => Some(data),
                _ => {
                    let ChildInfo {
                        dimension: _,
                        dimension_3d: _,
                        octant,
                    } = self.child_info(position).unwrap();

                    match self.children[octant as usize].deref() {
                        Some(child) => child.get(position),
                        _ => None,
                    }
                }
            };
        }

        None
    }

    /// Simplifies the `Node`.
    ///
    /// If all children are leaf `Node`s with identical data, destroy all children,
    /// and mark the `Node` as a leaf containing that data.
    pub(crate) fn simplify(&mut self) -> bool {
        let mut data = None;
        let mut miss_count = 0;

        if let NodeType::Leaf(_) = self.ty {
            return false;
        }

        for i in 0..OCTREE_CHILDREN {
            if let Some(child) = self.children[i].deref() {
                if !child.is_leaf() {
                    return false;
                }

                let leaf_data = child.leaf_data();

                if data.is_none() {
                    data = leaf_data;
                } else if data.unwrap() != leaf_data.unwrap() {
                    miss_count += 1;
                }
            } else {
                miss_count += 1;
            }
        }

        if miss_count > 1 {
            return false;
        }
        if miss_count == 1 && self.bounds[1].x - self.bounds[0].x > 2 {
            return false;
        }

        if data.is_some() {
            self.ty = NodeType::Leaf((*data.unwrap()).clone());
        }

        self.children.fill(Box::new(None));
        true
    }

    /// Returns a higher LOD of the current `Node`.
    ///
    /// For all children of a leaf `Node`, take the most common data of all children,
    /// destroy all children, and mark the `Node` as a leaf containing that data.
    pub(crate) fn lod(&mut self) {
        let mut all_data = [Default::default(); OCTREE_CHILDREN];
        for (i, c) in self.children.iter_mut().enumerate().map(|(i, c)| (i, c.deref_mut())) {
            if let Some(c) = c {
                if c.is_leaf() {
                    let leaf_data = c.leaf_data();

                    if leaf_data.is_some() {
                        all_data[i] = *leaf_data.unwrap();
                    }
                } else {
                    c.lod();
                }
            } else {
                return;
            }
        }

        let mut counts = HashMap::new();
        for data in all_data.iter() {
            counts.entry(*data).and_modify(|e| *e += 1).or_insert(1);
        }

        if !counts.is_empty() {
            let mut counts = counts.iter().collect::<Vec<(&T, &i32)>>();
            counts.sort_by(|a, b| b.1.cmp(a.1));

            self.ty = NodeType::Leaf(*counts[0].0);
        }

        self.children.fill(Box::new(None));
    }

    /// Returns the dimension of the `Node`.
    pub(crate) fn dimension(&self) -> u32 {
        (self.bounds[0].x as i32 - self.bounds[1].x as i32).abs() as u32
    }

    /// Returns whether the `Node` contains the given position.
    pub(crate) fn contains(&self, position: Vector3<u32>) -> bool {
        position.x >= self.bounds[0].x
            && position.x < self.bounds[1].x
            && position.y >= self.bounds[0].y
            && position.y < self.bounds[1].y
            && position.z >= self.bounds[0].z
            && position.z < self.bounds[1].z
    }

    /// Get leaf data from this `Node`.
    pub(crate) fn leaf_data(&self) -> Option<&T> {
        match &self.ty {
            NodeType::Leaf(data) => Some(&data),
            _ => None,
        }
    }

    fn child_info(&self, position: Vector3<u32>) -> Option<ChildInfo> {
        if self.contains(position) {
            let dimension = self.dimension() / 2;
            let dimension_3d = Vector3::from([dimension, dimension, dimension]);
            let midpoint = self.min_position() + dimension_3d;
            let octant = Octant::vector_diff(midpoint, position);

            Some(ChildInfo {
                dimension,
                dimension_3d,
                octant,
            })
        } else {
            None
        }
    }

    fn child_bounds(&self, dimension_3d: Vector3<u32>, octant: Octant) -> Bounds {
        let lower = self.min_position() + dimension_3d.component_mul(&octant.offset());
        let upper = lower + dimension_3d;

        [lower, upper]
    }

    fn child_count(&self) -> usize {
        self.children
            .iter()
            .fold(0, |acc, child| if child.deref().is_some() { acc + 1 } else { acc })
    }

    fn min_position(&self) -> Vector3<u32> {
        self.bounds[0]
    }

    fn is_leaf(&self) -> bool {
        matches!(self.ty, NodeType::Leaf(_))
    }
}
