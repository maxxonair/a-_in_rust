/*  =================================
 *        [A* Test Code ].mk0
 *  =================================
 *  Function: Creates (equidistant/constant cost) terrain of cells 
 *  and carries out A* path finding. 
 *  Terrain is drawn in the terminal. Symbols are:
 *    - (Space) free cell
 *  % - walls
 *  1 - Start point
 *  2 - End point
 *  = - Current path Element
 *  ^ - Current path Head
 *  TBD explored path (?)
 * 
 * Example terrain (30x45): 
 * " %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% "
// " %                                           % "
// " %                                           % "
// " %  2                                        % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                                           % "
// " %                            1              % "
// " %                                           % "
// " %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% "
 *
 */
use std::{thread, time};

// Test Grid dimensions
const GRID_NR_ROWS: usize = 30;
const GRID_NR_COLS: usize = 45;

/* Cell type constants */
const FREE_CELL: u32     = 0;
const BLOCKED_CELL: u32  = 1;
const START_POINT: u32   = 2;
const END_POINT: u32     = 3;
const PATH_ELEMENT: u32  = 4;
const PATH_HEAD: u32     = 5;
const PATH_EXPLORED: u32 = 6;

/* Create structure to store data for each grid point */
#[derive(Copy, Clone)]
struct GridPoint {
    id: u32,
    cell_type: u32,
    cell_coord: (usize, usize),
    parent_id: u32,
    is_start: u32,
    f: f32,
    g: f32,
    h: f32,
}

impl GridPoint {
    fn new() -> GridPoint {
        GridPoint {
            id: 0,
            cell_type: FREE_CELL,
            cell_coord: (0,0),
            parent_id: 0,
            is_start: 0,
            f: -1.0,
            g: -1.0,
            h: -1.0,
        }
    }
}

/* Function assign symbols to grid point types to plot in terminal */
fn create_gridpoint_string(point_id: u32) -> char {
    let mut return_char: char = ' ';
    if point_id == FREE_CELL {
        return_char = ' ';
    } else if point_id == BLOCKED_CELL {
        return_char = '%';
    } else if point_id == START_POINT {
        return_char = '1';
    } else if point_id == END_POINT {
        return_char = '2';
    } else if point_id == PATH_ELEMENT {
        return_char = '+';
    } else if point_id == PATH_HEAD {
        return_char = '^';
    } else if point_id == PATH_EXPLORED {
        return_char = '!';
    } else {
        // TODO: handle edge case
    }
    return_char
}

fn set_start_point(mut grid: Vec<Vec<GridPoint>>, coord: (usize, usize)) -> Vec<Vec<GridPoint>>{
    grid[coord.0][coord.1].cell_type = START_POINT;

    // TODO: Add check if start point is INSIDE the walls 
    grid
}

fn set_end_point(mut grid: Vec<Vec<GridPoint>>, coord: (usize, usize)) -> Vec<Vec<GridPoint>>{
    grid[coord.0][coord.1].cell_type = END_POINT;

    // TODO: Add check if end point is INSIDE the walls 
    grid
}

fn set_barrier_point(mut grid: Vec<Vec<GridPoint>>,  coord: (usize, usize)) -> Vec<Vec<GridPoint>>{
    grid[coord.0][coord.1].cell_type = BLOCKED_CELL;
    grid
}

fn find_cell_coord_to_id(grid: &Vec<Vec<GridPoint>>, id: u32) -> (usize, usize){
    let mut coord: (usize, usize) = (0,0);
    for ii in 0..GRID_NR_ROWS{
        for jj in 0..GRID_NR_COLS{
            if grid[ii][jj].id == id{
                coord = grid[ii][jj].cell_coord;
            }
        }
    }
    coord
}

fn is_id_in_set( vec: &Vec<GridPoint>, id: u32) -> u32 {
    for ii in 0..vec.len(){
        if vec[ii].id == id {
           return 1;
        }
    }
    return 0;
}

fn comp_h_score( point: GridPoint, end: GridPoint) -> f32 {
   let h: f32 = comp_heuristic_cost(point.cell_coord, end.cell_coord);
   h
}

/* Function: 
 * (1) Set outer boundary (Walls)
 * (2) Set inside cells to FREE_CELL
 * (3) Fill in cell coordincates
 *  */
fn set_boundary(mut grid: Vec<Vec<GridPoint>>) -> Vec<Vec<GridPoint>> {
    let mut cell_id: u32 = 1;
    for ii in 0..GRID_NR_ROWS{
        for jj in 0..GRID_NR_COLS{
            grid[ii][jj].id = cell_id;
            if ii == 0 || jj == 0 || ii == (GRID_NR_ROWS-1) || jj == (GRID_NR_COLS-1){
                grid[ii][jj].cell_type = BLOCKED_CELL;
            } else {
                grid[ii][jj].cell_type = FREE_CELL;
            }
            grid[ii][jj].cell_coord = (ii, jj);
            cell_id = cell_id + 1 ;
        }
    }
    grid
}

fn find_lowest_score_in_set( set: &Vec<GridPoint>) -> usize {
    /* Initialize index to point to the lowest f score in set */
    let mut index: usize = 0;
    for ii in (0..set.len()).rev(){
        if set[ii].f < set[index].f{
            index = ii;
        }
    }
    index
}

/* Function: Remove grid point with id from given set */
fn remove_grid_point_from_set( mut set: Vec<GridPoint>, id: u32) -> Vec<GridPoint> {
    for ii in (0..set.len()).rev(){
        if set[ii].id == id {
            set.remove(ii);
        }
    }
    set
}

/* Function compute heuristic cost, corresponding to a given start and end point coordinates */
fn comp_heuristic_cost(coord_cell: (usize, usize), coord_end: (usize, usize)) -> f32 {
    /* Convert all coordinates from u32 to f32 */
    let p1x: f32 = coord_cell.0 as f32;
    let p1y: f32 = coord_cell.1 as f32;
    let p2x: f32 = coord_end.0 as f32;
    let p2y: f32 = coord_end.1 as f32;
    /* compute heuristic cost to the end point */
    let heuristic_cost: f32 = f32::sqrt( ((p2x - p1x ).abs()).powf(2.0) + ((p2y - p1y ).abs()).powf(2.0));
    heuristic_cost
}

/* Print entire grid to terminal */
fn print_grid(grid: &Vec<Vec<GridPoint>>){
    let mut line = String::from(""); 
    let _ii: u32;
    let _jj: u32;
    let mut my_cell: char;
    
    for _ii in 0..GRID_NR_ROWS{
        line = String::from(" "); 
        for _jj in 0..GRID_NR_COLS{
            my_cell = create_gridpoint_string(grid[_ii][_jj].cell_type);
            line.push( my_cell );
        }
        line.push( ' ' );
        println!("{:?}", line);
    }
}

fn main() {

    println!(" ---- A* Test ----- ");

    //-----------------------------------------------------------------------------
    /*
     * Create and setup environment 
     */
    //-----------------------------------------------------------------------------
    let mut grid_environment: Vec<Vec<GridPoint>> = vec![vec![GridPoint::new(); GRID_NR_COLS]; GRID_NR_ROWS];


    /* Set Walls */
    grid_environment = set_boundary( grid_environment );
    /* Set Start Point */
    let start_point: (usize, usize) = (27, 20);
    grid_environment = set_start_point(grid_environment,  start_point);
    /* Set cell flag: This is the start point */
    grid_environment[start_point.0][start_point.1].is_start = 1;

    /* Set End Point */
    let end_point: (usize, usize) = (3, 3);
    grid_environment = set_end_point(grid_environment,  end_point);

    /* Set barriers */
    for ii in 1..(GRID_NR_COLS-6){
        grid_environment = set_barrier_point(grid_environment,  (16, ii));
    }



    //-----------------------------------------------------------------------------
    /* Initialize status falgs */

    /* Init flag: If = 1 -> end reached */
    let mut end_reached: u32 = 0;

    /* Init flag: If = 1 -> end not reachable */ 
    let mut end_not_reachable: u32 = 0;
    //-----------------------------------------------------------------------------
    /*
     *      Run Path Search Sequence 
     */
    //-----------------------------------------------------------------------------
    /* Additional boundary for debugging */
    let mut debug_counter: u32 = 0;
    let debug_stop: u32 = 1000;

    /* Create sets  */
    let mut open_set: Vec<GridPoint> = Vec::new();
    open_set.push(grid_environment[start_point.0][start_point.1]);

    let mut closed_set: Vec<GridPoint> = Vec::new();

    // Loop as long as end is reached or end not reachable flag true
    while end_reached == 0 && end_not_reachable == 0 && debug_counter < debug_stop{
        /* Clear Terminal */
        print!("{}[2J", 27 as char);

        if open_set.len() > 0 {
            /* A* code  */
            // TBDs
            let index: usize = find_lowest_score_in_set(&open_set);
            /* Get currently focused grid point  */
            let mut current_point: GridPoint = open_set[index];
            
            if open_set[index].id == grid_environment[end_point.0][end_point.1].id {
                /* End condition reached -> Great Success */
                end_reached = 1;
                println!(" >> End point reached. Great Success! Exiting.");

                /* Find optimal path */

                /* Create Optimal path list */
                let mut optimal_path: Vec<GridPoint> = Vec::new();
                /* Push current point (we should equal the end point at this stage) */
                optimal_path.push(current_point);

                println!("");
                println!("[Computing Optimal Path] ... ");
                /* Loop backwards until start point is reached */
                while current_point.is_start == 0 {
                    /* Print for debug only  */
                    println!(" id {:?} :: pid {:?} :: {:?}", current_point.id, current_point.parent_id, current_point.cell_coord);
                    /* Push parent cell to optimal path */
                    let cx = find_cell_coord_to_id(&grid_environment, current_point.parent_id);
                    optimal_path.push(grid_environment[cx.0][cx.1]);
                    /* Set parent cell as current cell  */
                    current_point = grid_environment[cx.0][cx.1];
                }

                /* Plot Computed path */
                for jj in 0..optimal_path.len(){
                    let cm = optimal_path[jj].cell_coord;
                    grid_environment[cm.0][cm.1].cell_type = PATH_ELEMENT;
                }

            } else {
                /* Remove current grid point from open set  */
                open_set = remove_grid_point_from_set(open_set, current_point.id);
                /* Add curent point to closed set */
                closed_set.push(current_point);

                /* If ID not equal to start or end point -> Update symbol for console drawing */
                if current_point.id != grid_environment[start_point.0][start_point.1].id && 
                    current_point.id != grid_environment[start_point.0][start_point.1].id
                    {
                        let co: (usize, usize) = find_cell_coord_to_id(&grid_environment, current_point.id);
                        grid_environment[co.0][co.1].cell_type = PATH_EXPLORED;
                    }

                    /* Add new neighbours to set */
                    let straight_cost: f32 = 1.0;
                    /* List holding only empty/traversable grid cell coordinates */
                    let mut neighbour_coord = Vec::new();
                    /* List holding complete set of neighbouring grid cell coordinates */
                    let mut neighbour_full = Vec::new();
                    let cc = current_point.cell_coord;

                    neighbour_full.push((cc.0 - 1, cc.1 - 1));
                    neighbour_full.push((cc.0 + 1, cc.1 - 1));
                    neighbour_full.push((cc.0 - 1, cc.1 + 1));
                    neighbour_full.push((cc.0 + 1, cc.1 + 1));
                    neighbour_full.push((cc.0    , cc.1 - 1));
                    neighbour_full.push((cc.0    , cc.1 + 1));
                    neighbour_full.push((cc.0 - 1, cc.1    ));
                    neighbour_full.push((cc.0 + 1, cc.1    ));

                    for kk in 0..neighbour_full.len()
                    {
                        let c = neighbour_full[kk];
                        if grid_environment[c.0][c.1].cell_type != BLOCKED_CELL{
                            neighbour_coord.push(c);
                        }
                    }

                    for ii in 0..neighbour_coord.len()
                    {
                        /* Compute neighbour coordinates */
                        let nc = neighbour_coord[ii];
                        /* Compute tentative g score  */
                        let tentative_g: f32 = current_point.g + straight_cost;
                        
                        
                        /* If new g score is lower than the previous or cell has not yet been evaluated */
                        if tentative_g < grid_environment[nc.0][nc.1].g || grid_environment[nc.0][nc.1].g == -1.0 {
                            
                            grid_environment[nc.0][nc.1].g = tentative_g;
                            grid_environment[nc.0][nc.1].h = comp_h_score(grid_environment[nc.0][nc.1], grid_environment[end_point.0][end_point.1]);
                            grid_environment[nc.0][nc.1].f = grid_environment[nc.0][nc.1].g + grid_environment[nc.0][nc.1].h;
                            grid_environment[nc.0][nc.1].parent_id = current_point.id;
                            
                            if is_id_in_set( &open_set, grid_environment[nc.0][nc.1].id) == 0 {
                                open_set.push(grid_environment[nc.0][nc.1]);
                            }
                        }
                    }

            }
    
        } else {
            // End condition reached -> no solution 
            end_not_reachable = 1;
            println!(" >> End point not reachable. No solution obtained.");
        }

        /* Plot Status */
        println!("Itertation step #{:?}", debug_counter);
        println!(" ");
        print_grid( &grid_environment );
        println!(" ");
        thread::sleep(time::Duration::from_millis(10));

        /* Update debug counter */
        debug_counter = debug_counter + 1 ;
    }

}