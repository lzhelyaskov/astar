Generic implementation of A* (A Star) algorithm in rust.
========================================================
It is poorly tested, unit tests are a 'todo'.

Usage
-----
```rust
use astar::*;

const WIDTH: i32 = 12;
const HEIGHT: i32 = 12;


fn main() {
    // 12x12
	// 's' is start node. 'g' is goal. 'x' are "walls".
    #[cfg_attr(rustfmt, rustfmt_skip)]
    let map = [
        ' ', ' ', ' ', ' ', ' ', 'x', ' ', ' ', ' ', ' ', ' ', 'g', 
        ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 
        ' ', ' ', 'x', 'x', ' ', ' ', ' ', 'x', 'x', 'x', ' ', ' ', 
        ' ', ' ', 'x', 'x', ' ', 'x', ' ', ' ', ' ', 'x', ' ', ' ', 
        ' ', ' ', 'x', ' ', ' ', 'x', ' ', ' ', ' ', 'x', ' ', ' ', 
        ' ', ' ', ' ', ' ', ' ', 'x', 'x', 'x', ' ', 'x', ' ', ' ', 
        ' ', ' ', ' ', 'x', ' ', ' ', ' ', 'x', ' ', 'x', ' ', ' ', 
        ' ', ' ', ' ', 'x', ' ', ' ', ' ', ' ', ' ', 'x', ' ', ' ', 
        ' ', ' ', ' ', 'x', ' ', ' ', 'x', 'x', ' ', 'x', ' ', ' ', 
        ' ', ' ', ' ', ' ', ' ', ' ', 'x', ' ', ' ', 'x', ' ', ' ', 
        ' ', ' ', 'x', ' ', ' ', ' ', 'x', ' ', ' ', ' ', ' ', ' ', 
        's', ' ', 'x', ' ', ' ', ' ', 'x', ' ', ' ', ' ', ' ', ' ', 
    ];

    let goal = (11, 0);
    let start = (0i32, 11i32);
    let reachable = | node: (i32, i32) | neigbours(&node, &map).into_iter();
    let is_goal = | node: (i32, i32) | node == goal;
    let goal_cost = | a | manhattan(a, goal);
    let edge_cost = | _, _ | 1f32;

    if let Some(path) = astar(start, reachable, is_goal, goal_cost, edge_cost) {
        for x in path {
            println!("{:?}", x)
        }
    } else {
        println!("no path found.")
    }
}

fn neigbours(node: &(i32, i32), map: &[char]) -> Vec<(i32, i32)> {
    const GRID_TEMPLATE: [(i32, i32); 8] = [
        (-1, 1),
        (0, 1),
        (1, 1),
        (-1, 0),
        (1, 0),
        (-1, -1),
        (0, -1),
        (1, -1),
    ];

    let (x, y) = *node;
    let mut reachable = Vec::new();

    for t in GRID_TEMPLATE {
        let xx = x + t.0;
        let yy = y + t.1;

        if xx < 0 || xx >= WIDTH {
            continue;
        }

        if yy < 0 || yy >= HEIGHT {
            continue;
        }

        let idx = yy * WIDTH + xx;

        if map[idx as usize] != 'x' {
            reachable.push((xx, yy));
        }
    }

    reachable
}

pub fn manhattan(start: (i32, i32), goal: (i32, i32)) -> f32 {
    let (x1, y1) = start;
    let (x2, y2) = goal;

    ((x1 - x2).abs() + (y1 - y2).abs()) as f32
}
```
running this produces:

(11, 0)
(10, 0)
(9, 0)
(8, 0)
(7, 1)
(6, 2)
(7, 3)
(8, 4)
(8, 5)
(8, 6)
(7, 7)
(6, 6)
(5, 7)
(4, 8)
(3, 9)
(2, 9)
(1, 10)

as you can see, the order is reversed and start node is not included

common heuristics
-----------------
```rust
/// h=∣xstart−xdestination∣+∣ystart−ydestination∣
pub fn manhattan(start: (u32, u32), goal: (u32, u32)) -> f32 {
    let (x1, y1) = start;
    let (x2, y2) = goal;

    ((x1 as i32 - x2 as i32).abs() + (y1 as i32 - y2 as i32).abs()) as f32
}

/// h = max { abs(current_cell.x – goal.x), abs(current_cell.y – goal.y) }
pub fn diagonal(start: (u32, u32), goal: (u32, u32)) -> f32 {
    let (x1, y1) = start;
    let (x2, y2) = goal;

    ((x1 as i32 - x2 as i32)
        .abs()
        .max((y1 as i32 - y2 as i32).abs())) as f32
}

/// h= sqrt( (xstart−xdestination)2+(ystart−ydestination)2 )
pub fn euclidean(start: (u32, u32), goal: (u32, u32)) -> f32 {
    let (x1, y1) = start;
    let (x2, y2) = goal;

    (((x1 as i32 - x2 as i32) * 2 + (y1 as i32 - y2 as i32) * 2) as f32).sqrt()
}
```

todo:
-----
* write unit tests
* think about moving collections out of the *astar* function 