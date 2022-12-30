use std::{
    cmp::Reverse,
    collections::{BinaryHeap, HashMap, HashSet},
    hash::Hash,
};

pub trait NodeId: Eq + Hash + Copy {}

impl <T: Eq + Hash + Copy> NodeId for T {}

#[derive(Debug)]
struct NodeInfo<Tid: NodeId> {
    f: f32,
    g: f32,
    parent: Tid,
}

impl<Tid: NodeId> NodeInfo<Tid> {
    /// g: calculated cost from start to this node
    /// h: estimated cost from this node to the goal
    fn new(g: f32, h: f32, parent: Tid) -> Self {
        NodeInfo {
            g,
            f: g + h,
            parent,
        }
    }
}

#[derive(PartialEq, Debug)]
struct OpenListElement<Tid: NodeId> {
    f: f32,
    id: Tid,
}

impl<Tid: NodeId> OpenListElement<Tid> {
    fn new(f: f32, id: Tid) -> Self {
        OpenListElement { f, id }
    }
}

impl<Tid: NodeId> Eq for OpenListElement<Tid> {}

impl<Tid: NodeId> PartialOrd for OpenListElement<Tid> {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        self.f.partial_cmp(&other.f)
    }
}

impl<Tid: NodeId> Ord for OpenListElement<Tid> {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap() // <-- todo?
    }
}

#[derive(Debug)]
pub struct Path<Tid: NodeId> {
    to: Tid,
    next_id: Tid,
    // reuses 'infos' HashMap from the astar function
    // contains all examined nodes, not just the relevant ones
    infos: HashMap<Tid, NodeInfo<Tid>>,
}

impl<Tid: NodeId> Path<Tid> {
    fn new(infos: HashMap<Tid, NodeInfo<Tid>>, start: Tid, finish: Tid) -> Self {
        Path {
            to: finish,
            next_id: start,
            infos,
        }
    }
    // todo: a trim() method? 
    // (remove nodes from self.infos which are not involved in an actual path)
}

impl<Tid: NodeId> Iterator for Path<Tid> {
    type Item = Tid;

    fn next(&mut self) -> Option<Self::Item> {
        if self.next_id == self.to {
            None
        } else {
            let result = self.next_id;
            self.next_id = self.infos[&self.next_id].parent;
            Some(result)
        }
    }
}

/// start: node id to start from
/// reachable: returns list of neighbours/nodes reachable from param node
/// is_goal: is the param node a goal/destination? (end search)
/// goal_cost: estimated distance from node to goal/destination
/// edge_cost: transition cost between two neighbouring nodes
pub fn astar<Tid: NodeId, Fr, Ti: Iterator<Item = Tid>, Fg, Fh, Fc>(
    start: Tid,
    reachable: Fr,
    is_goal: Fg,
    goal_cost: Fh,
    edge_cost: Fc,
) -> Option<Path<Tid>>
where
    Fg: Fn(Tid) -> bool,
    Fh: Fn(Tid) -> f32,
    Fr: Fn(Tid) -> Ti,
    Fc: Fn(Tid, Tid) -> f32,
{
    if is_goal(start) {
        return None;
    }

    let mut closed = HashSet::new();
    let mut open = BinaryHeap::new();
    let mut infos = HashMap::new();

    infos.insert(start, NodeInfo::new(0f32, 0f32, start));
    open.push(Reverse(OpenListElement::new(0f32, start)));

    while let Some(Reverse(element)) = open.pop() {
        let parent_id = element.id;
        closed.insert(parent_id);
        let neighbours = reachable(parent_id);

        for neighbour_id in neighbours {
            if is_goal(neighbour_id) {
                dbg!(open.len(), closed.len(), infos.len());
                infos.insert(neighbour_id, NodeInfo::new(0f32, 0f32, parent_id));
                return Some(Path::new(infos, neighbour_id, start));
            }

            if closed.contains(&neighbour_id) {
                continue;
            }

            let node_info = NodeInfo::new(
                infos[&parent_id].g + edge_cost(parent_id, neighbour_id),
                goal_cost(neighbour_id),
                parent_id,
            );

            if !infos.contains_key(&neighbour_id) || infos[&neighbour_id].f > node_info.f {
                open.push(Reverse(OpenListElement::new(node_info.f, neighbour_id)));
                infos.insert(neighbour_id, node_info);
            }
        }
    }
    None
}