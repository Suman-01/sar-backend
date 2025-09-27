// Represent the search area as a grid of tiles
// Track which tiles have been visited/unvisited
// Provide APIs for updating coverage , computing search progress (in %)


#[derive(Debug, Clone)]
// Single Cell/Tile in Grid | Whether Tile visited or not
pub struct Cell {
    pub row: u32,   // Integer Rows
    pub col: u32,   // Integer Cols
    pub visited: bool,
    pub confidence: f32,
}

impl Cell {
    pub fn new_cell(row: u32, col: u32) -> Self {
        Self {
            row,
            col,
            visited: false,
            confidence: 0.0,    // min - 0, max - 1
        }
    }

    pub fn mark_visited(&mut self) {
        self.visited = true;
        self.confidence = 1.0;  // if visited, conf <- 1
    }

    pub fn update_confidence(&mut self, value: f32) {
        self.confidence = value.clamp(0.0, 1.0);    // clamp the value b/w 0 and 1

        // Strict visited mark | No threshold Implemented | Also keep it as binary flag for operator to flip 
        if self.confidence == 1.0 {     
            self.visited = true;
        }
    }
}

// Grid --> Full Rectangular Search Area
pub struct Grid {
    rows: u32,     // Integer Rows
    cols: u32,     // Integer Cols
    cells: Vec<Vec<Cell>>,
}

impl Grid {
    pub fn new_grid(rows: u32, cols: u32) -> Self {
        let (rows_usize, cols_usize) = (rows as usize, cols as usize);  // Type casted only for indexing
        let mut cells = Vec::with_capacity(rows_usize);

        for r in 0..rows {
            let mut row = Vec::with_capacity(cols_usize);
            for c in 0..cols {
                row.push(Cell::new_cell(r as u32, c as u32));   // u32 based Cells in each col
            }
            cells.push(row);    // Rows of u32 based Cells
        }
        Self {
            rows,
            cols,
            cells,
        }
    }

    pub fn rows(&self) -> u32 {
        self.rows
    }

    pub fn cols(&self) -> u32 {
        self.cols
    }

    pub fn get_cell(&self, rows: u32, cols: u32) -> Option<&Cell> {
        self.cells.get(rows as usize).and_then(|r| r.get(cols as usize))
    }

    pub fn get_cell_mut(&mut self, rows: u32, cols: u32) -> Option<&mut Cell> {
        self.cells.get_mut(rows as usize).and_then(|r| r.get_mut(cols as usize))
    }

    pub fn visit_gridcell(&mut self, rows: u32, cols: u32) {
        if let Some(cell) = self.get_cell_mut(rows, cols) {
            cell.mark_visited();
        }
    }

    pub fn update_gridcell_confidence(&mut self, rows: u32, cols: u32, confidence: f32) {
        if let Some(cell) = self.get_cell_mut(rows, cols) {
            cell.update_confidence(confidence);
        }
    }

    pub fn grid_portion_covered(&self) -> f32 {
        let mut visited_cnt: u32 = 0;
        let mut total_cnt: u32 = 0;

        for row in &self.cells {
            for cell in row {
                total_cnt += 1;
                if cell.visited {
                    visited_cnt += 1;
                }
            }
        }

        if total_cnt == 0 {
            0.0
        } else {
            visited_cnt as f32/ total_cnt as f32
        }
    }
}


//Integer Rows and Cols might change to float
//Type Casting Changes | Runtime issues
//Entire Structure of Cell and Grid is in u32
//Encapsulation not done | If you encapsulate the values how do you manage access | don't need manager for every file