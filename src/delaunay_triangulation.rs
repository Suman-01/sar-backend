pub const NULL_INDEX: usize = usize::MAX;

# [derive(Clone, Copy)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

struct QuarterEdge {
    nxt: usize, // next ccw edge from origin
    origin_idx: usize,
}

pub struct Mesh {
    edges: Vec<QuarterEdge>,
    points: Vec<Point>,
}

impl Mesh {
    pub fn new_mesh(points: Vec<Point>) -> Self {
        let num_edges = points.len() * 4;
        Mesh { 
            edges: (Vec::with_capacity(num_edges)), 
            points,
        }
    }

    // Navigators
    // counter-clockwise dual edge
    pub fn rot(&self, edge: usize) -> usize {
        if edge & 3 == 3 {edge - 3} else {edge + 1}
    }
    // reverse edge
    pub fn sym(&self, edge: usize) -> usize {
        if edge & 3 >= 2 {edge - 2} else {edge + 2}
    }
    // reverse dual edge
    pub fn tor(&self, edge: usize) -> usize {
        if edge & 3 == 0 {edge + 3} else {edge - 1}
    }
    // next ccw edge from origin 
    pub fn onext(&self, edge: usize) -> usize {
        self.edges[edge].nxt
    }
    // next cw edge from origin 
    pub fn oprev(&self, edge: usize) -> usize {
        self.rot(self.onext(self.rot(edge)))
    }
    // next cw edge from sym edge
    pub fn dnext(&self, edge: usize) -> usize {
        self.rot(self.onext(self.tor(edge)))
    }
    // next ccw edge from sym edge
    pub fn dprev(&self, edge: usize) -> usize {
        self.onext(self.sym(edge))
    }
    // origin point index of the edge
    pub fn org(&self, edge: usize) -> usize {
        self.edges[edge].origin_idx
    }
    // destination point index of the edge
    pub fn dest(&self, edge: usize) -> usize {
        self.edges[self.sym(edge)].origin_idx
    }

    pub fn make_edge(&mut self, origin_idx: usize, dest_idx: usize) -> usize {
        let edge_idx = self.edges.len();

        self.edges.push(QuarterEdge { nxt: (edge_idx), origin_idx: origin_idx}); // e points itself initially (AB)
        self.edges.push(QuarterEdge { nxt: (edge_idx + 3), origin_idx: 0 }); // rot points to tor (sincle initially not closed)
        self.edges.push(QuarterEdge { nxt: (edge_idx + 2), origin_idx: dest_idx }); // sym points itself initially (BA)
        self.edges.push(QuarterEdge { nxt: (edge_idx + 1), origin_idx: 0 }); // tor points to rot (sincle initially not closed)

        edge_idx
    }

    // i don't intuitively understand this 
    pub fn splice (&mut self, a: usize, b: usize) {
        let alpha = self.onext(a);
        let beta = self.onext(b);
        self.edges[a].nxt = beta;
        self.edges[b].nxt = alpha;

        let alpha_rot = self.rot(alpha);
        let beta_rot = self.rot(beta);
        let alpha_rot_next = self.edges[alpha_rot].nxt;
        let beta_rot_next = self.edges[beta_rot].nxt;

        self.edges[alpha_rot].nxt = beta_rot_next;
        self.edges[beta_rot].nxt = alpha_rot_next;
    }

    // connect a and b, and return the edge from a to b (dest_edge, origin_edge)
    pub fn connect(&mut self, a: usize, b: usize) -> usize {
        let e = self.make_edge(self.dest(a), self.org(b));
        self.splice(e, self.dnext(a));  // i don't understand this 
        self.splice(self.sym(e), b);  // i don't understand this
        e
    }

    pub fn delete_edge(&mut self, e: usize) {
        self.splice(e, self.oprev(e));  // no intution on this either
        self.splice(self.sym(e), self.oprev(self.sym(e)));  // no intution on this either

        self.edges[e].origin_idx = NULL_INDEX;
        let temp = self.sym(e);
        self.edges[temp].origin_idx = NULL_INDEX;
    }

    pub fn get_edges_for_plotting(&self) -> Vec<((f64, f64), (f64, f64))> {
        let mut lines = Vec::new();

        for i in (0..self.edges.len()).step_by(4) {
            let o = self.edges[i].origin_idx;
            let d = self.edges[self.sym(i)].origin_idx;

            if o == NULL_INDEX {
                continue;
            }

            let p_o = self.points[o];
            let p_d = self.points[d];

            lines.push(((p_o.x, p_o.y), (p_d.x, p_d.y)));
        }
        lines
    }

}

// MATH
// cross - prod ab and ac if c is ccw to ab or cw
fn ccw(a: Point, b: Point, c: Point) -> bool {
    (b.x - a.x) * (c.y - a.y) > (c.x - a.x) * (b.y - a.y)
}

// in-circle test, if d is inside the circumcircle of abc (Guibas & Stolfi determinant test)
// det(matrix):
// | ax-dx   ay-dy   alift |
// | bx-dx   by-dy   blift |
// | cx-dx   cy-dy   clift |
fn inside_circle(a: Point, b: Point, c: Point, d: Point, ) -> bool {
    // lift points by d
    let ax = a.x - d.x; let ay = a.y - d.y;
    let bx = b.x - d.x; let by = b.y - d.y;
    let cx = c.x - d.x; let cy = c.y - d.y;

    let alift = ax * ax + ay * ay;
    let blift = bx * bx + by * by;
    let clift = cx * cx + cy * cy;

    let jk = by * clift - cy * blift;
    let ik = bx * clift - cx * blift;
    let ij = bx * cy - cx * by;

    return (ax * jk - ay * ik + alift * ij) > 0.0

}

// Validity check: 
fn is_valid(mesh: &Mesh, cand: usize, base: usize) -> bool {
    ccw(
        mesh.points[mesh.dest(cand)],
        mesh.points[mesh.dest(base)],
        mesh.points[mesh.org(base)]
    )
}

//indices is the list of point indices | returns edge indices 
fn delaunay_recursive(mesh: &mut Mesh, indices: &[usize]) -> (usize, usize) {
    let n = indices.len();

    // if only 2 points, connect them and return the edge and its sym edge
    if n == 2 {
        let e  = mesh.make_edge(indices[0], indices[1]);
        return(e, mesh.sym(e));
    }

    if n == 3 {
        let ab = mesh.make_edge(indices[0], indices[1]);
        let bc = mesh.make_edge(indices[1], indices[2]);
        mesh.splice(mesh.sym(ab), bc);

        let p0 = mesh.points[indices[0]];
        let p1 = mesh.points[indices[1]];
        let p2 = mesh.points[indices[2]];

        if ccw(p0, p1, p2) {
            let _ca = mesh.connect(bc, ab);
            return (ab, mesh.sym(bc));
        } else if ccw(p0, p2, p1) {
            let ca = mesh.connect(bc, ab);
            return (mesh.sym(ca), ca);
        } else {
            return (ab, mesh.sym(bc));
        }
    }

    // Divide
    let mid = n / 2;
    let (l_out, l_in) = delaunay_recursive(mesh, &indices[..mid]); // l_out: left delaunay outer edge, l_in: left delaunay inner edge
    let (r_in, r_out) = delaunay_recursive(mesh, &indices[mid..]); // r_in: right delaunay inner edge, r_out: right delaunay outer edge

    // Find lower common trangent of left and right triangulation
    let mut l_in = l_in;
    let mut r_in = r_in;

    loop {
        if ccw(mesh.points[mesh.org(r_in)], mesh.points[mesh.org(l_in)], mesh.points[mesh.dest(l_in)]) {
            l_in = mesh.dnext(l_in);
        } else if ccw(mesh.points[mesh.org(l_in)], mesh.points[mesh.dest(r_in)], mesh.points[mesh.org(r_in)]) {
            r_in = mesh.dprev(r_in);
        } else {
            break;
        }
    }

    let mut base = mesh.connect(mesh.sym(r_in), l_in);

    let l_out = if mesh.org(l_in) == mesh.org(l_out) { mesh.sym(base) } else {l_out};
    let r_out = if mesh.org(r_in) == mesh.org(r_out) { base } else {r_out};

    // -- ZIPPER --
    loop {
        let mut l_cand = mesh.dprev(base);
        let mut r_cand = mesh.oprev(base);
        let is_l_valid = is_valid(mesh, l_cand, base);
        let is_r_valid = is_valid(mesh, r_cand, base);

        if !is_l_valid && !is_r_valid {
            break;
        }

        if is_l_valid {
            while is_valid(mesh, mesh.onext(l_cand), base) && inside_circle(
                mesh.points[mesh.dest(l_cand)], 
                mesh.points[mesh.dest(base)], 
                mesh.points[mesh.org(base)], 
                mesh.points[mesh.dest(mesh.onext(l_cand))]
            ) {
                let temp = mesh.onext(l_cand);
                mesh.delete_edge(l_cand);
                l_cand = temp;
            }
        }

        if is_r_valid {
            while is_valid(mesh, mesh.oprev(r_cand), base) && inside_circle(
                mesh.points[mesh.dest(r_cand)], 
                mesh.points[mesh.dest(base)], 
                mesh.points[mesh.org(base)], 
                mesh.points[mesh.dest(mesh.oprev(r_cand))]
            ) {
                let temp = mesh.oprev(r_cand);
                mesh.delete_edge(r_cand);
                r_cand = temp;
            }
        }

        if !is_r_valid || (is_l_valid && inside_circle(
            mesh.points[mesh.dest(r_cand)], 
            mesh.points[mesh.dest(base)], 
            mesh.points[mesh.org(base)], 
            mesh.points[mesh.dest(l_cand)]
        )) {
            base = mesh.connect(mesh.sym(base), mesh.sym(l_cand));
        } else {
            base = mesh.connect(r_cand, mesh.sym(base));
        }
    }

    (l_out, r_out)
}

pub fn triangulate(points: Vec<(f64, f64)>) -> Mesh {
    let seeds: Vec<Point> = points.into_iter().map(|(x, y)| Point { x, y }).collect();
    let mut valid_points: Vec<Point> = seeds.into_iter().filter(|p| !p.x.is_nan() && !p.y.is_nan()).collect();
    valid_points.sort_by(|p1, p2| {
        p1.x.partial_cmp(&p2.x).unwrap().then(p1.y.partial_cmp(&p2.y).unwrap())
    });
    valid_points.dedup_by(|p1, p2| p1.x == p2.x && p1.y == p2.y);
    let mut mesh = Mesh::new_mesh(valid_points.clone());
    let indices: Vec<usize> = (0..valid_points.len()).collect();

    delaunay_recursive(&mut mesh, &indices);
    mesh 
}

