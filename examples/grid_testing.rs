use sar_backend::coverage::grid::{Grid};

fn main() {
    let mut grid = Grid::new_grid(5, 5);
    println!("Grid initialized with {} rows and {} cols", grid.rows(), grid.cols());

    for row in 0..5 {
        for col in 0..row + 1 {
            grid.visit_gridcell(row, col);
        }
    }

    grid.update_gridcell_confidence(0, 4, 1.1);
    grid.update_gridcell_confidence(2, 3, 0.5);

    println!("Grid portion Covered = {:.2}%", grid.grid_portion_covered() * 100.0);

    println!("Grid Visit status view: ");
    for row in 0..grid.rows() {
        for col in 0..grid.cols() {
            let cell = grid.get_cell(row, col).unwrap();
            let symbol = if cell.visited {"✔"} else {"·"};
            print!("{} ", symbol);
        }
        println!();
    }

}