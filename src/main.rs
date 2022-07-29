use std::{
    fs::File, io::BufReader,
};

use anyhow::Result;
use geo_clipper::Clipper;
use petgraph::{
    graph::UnGraph,
};
use plotters::{
    prelude::*,
    style::full_palette::{BLACK, GREEN_700},
};
mod ga;
use ga::{evaluate, Enviroment, Individual, Obstacles};
// use ga::GeneticAlgorithm;
use geo::{
    coord, Coordinate, CoordsIter, EuclideanLength, Intersects, Line, LineString,
    MultiPolygon, Polygon,
};

use rand::{distributions::Uniform, prelude::Distribution};
use rayon::{prelude::*};
use wkt::ToWkt;

mod tests;
fn main() -> Result<()> {
    // let file = OpenOptions::new().read(true).open("enviroment.json");
    // let polygons = MultiPolygon::from(_);
    // let test_poly = polygon!((x: 10., y: 10.), (x: 20., y: 10.), (x: 20., y: 20.), (x: 10., y: 20.));
    // let multi_poly = MultiPolygon::new(vec![test_poly.clone(), test_poly.clone(), test_poly.clone()]);
    // println!("MultiPolygon =\n{}", serde_json::to_string(&multi_poly)?);
    // let test_line = line_string![(x: 5., y: 0.), (x: 30., y: 35.)];
    // let ewagfawf = MultiLineString::from(test_line.clone());
    // let result = ewagfawf.intersection(&test_poly, 1.);
    // let line_on_edge = line_string![(x: 10.,y: 10.), (x: 20., y:20.)];
    // let res = test_poly.contains(&test_line);
    // println!("??? = {:?}, intersects = {}", result, res);
    let (obstacles, _enviroment) = read_enviroment_from_file("env.json")?;
    // let generation = 0;
    // let population_size = 100;
    // let mut population: Vec<Individual> = Vec::new();
    // population.reserve(population_size);
    // initialize_population(&mut population, &enviroment);
    // evaluate_population(&mut population, &obstacles);

    // let coords: Vec<_> = obstacles.static_obstacles.0.first().unwrap().exterior().0.iter().take(2).collect();
    // let line = Line::new(*coords[0], *coords[1]);
    // let res = line.intersects(&obstacles.static_obstacles);
    // println!("intersects = {res}");

    // let slice = [1,2,3,4,5,6];
    // let mut test_vec = Vec::new();

    // test_vec.extend_from_slice(&slice);
    // test_vec.extend_from_slice(&slice);
    // test_vec.extend_from_slice(&slice);
    // println!("test_vec: {test_vec:?}");
    println!("{}", obstacles.static_obstacles.to_wkt());

    let test_path: Vec<Coordinate<f64>> = vec![
        (10., 10.).into(),
        (200., 100.).into(),
        (600., 500.).into(),
        (200., 800.).into(),
    ];
    
    let fitness = evaluate(&Individual::new(test_path.clone()), &obstacles);
    draw_env_to_file("debug.png", &obstacles, &test_path)?;
    println!("total length {:.2}", fitness);
    Ok(())
}

fn initialize_population(population: &mut [Individual], enviroment: &Enviroment) {
    population.par_iter_mut().for_each(|x| {
        let between_width = Uniform::new(0., enviroment.width);
        let between_height = Uniform::new(0., enviroment.height);
        let mut rng = rand::thread_rng();
        x.points.reserve(10);
        for point in x.points.iter_mut() {
            point.x = between_width.sample(&mut rng);
            point.y = between_height.sample(&mut rng);
        }
    });
}

fn read_enviroment_from_file(filename: &str) -> Result<(Obstacles, Enviroment)> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);
    let parsed_json: serde_json::Value = serde_json::from_reader(reader)?;
    let env = Enviroment {
        width: parsed_json["width"].as_f64().unwrap_or_default(),
        height: parsed_json["height"].as_f64().unwrap_or_default(),
    };
    let mut polygons = vec![];
    let array: Vec<Vec<Vec<f64>>> =
        serde_json::from_value(parsed_json["static_obstacles"].clone())?;
    for polygon in array.iter() {
        let test: LineString = polygon
            .iter()
            .map(|coords| {
                coord! {x: coords[0], y: coords[1]}
            })
            .collect();
        polygons.push(Polygon::new(test, vec![]));
    }
    let multi_polygon = MultiPolygon::new(polygons);
    let multi_polygon_with_offset = multi_polygon.offset(
        3.,
        geo_clipper::JoinType::Miter(3.),
        geo_clipper::EndType::ClosedPolygon,
        1.,
    );
    let visibility_graph =
        build_visibility_graph_from_polygons(&multi_polygon, &multi_polygon_with_offset).unwrap();
    let obstacles = Obstacles {
        static_obstacles: multi_polygon.clone(),
        static_obstacles_with_offset: multi_polygon_with_offset.clone(),
        visibility_graph: visibility_graph,
    };
    Ok((obstacles, env))
}
fn evaluate_population(population: &mut Vec<Individual>) {}

// fn custom_collision_detection(polygons: &MultiPolygon<f64>, path: &MultiLineString<f64>) -> Option<MultiLineString<f64>>{
//     for polygon in polygons.iter(){
//         if polygon.intersects(path) {
//             let inner = path.iter().nth(0)?;
//             for line in inner.lines(){
//                 for polygon_line in polygon.lines_iter(){
//                     let intersection = line_intersection(line, polygon_line);
//                     let intersection = match intersection{
//                         Some(result) => result,
//                         None => continue,
//                     };
//                     let point = match intersection {
//                         LineIntersection::SinglePoint { intersection, is_proper } => intersection,
//                         LineIntersection::Collinear { intersection } => continue,
//                     };

//                 }
//             }
//         }
//     }
//     None
// }

fn draw_env_to_file(
    filename: &str,
    obstacles: &Obstacles,
    path: &Vec<Coordinate<f64>>,
) -> Result<()> {
    use plotters::prelude::Polygon;
    let root = BitMapBackend::new(filename, (1000, 1000)).into_drawing_area();
    root.fill(&WHITE)?;
    // Draw static obstacles with dark green color
    let polygon_color = ShapeStyle {
        color: GREEN_700.to_rgba().mix(0.7),
        filled: true,
        stroke_width: 2,
    };
    for poly in obstacles.static_obstacles.iter() {
        let drawing_poly: Vec<_> = poly
            .exterior()
            .coords_iter()
            .map(|point| (point.x as i32, point.y as i32))
            .collect();
        // println!("{drawing_poly:?}");
        let polygon = Polygon::new(drawing_poly, polygon_color);
        root.draw(&polygon)?;
    }
    // Draw outline of offset polygon in blue
    let offset_polygon_color = ShapeStyle {
        color: BLUE.mix(0.7),
        filled: false,
        stroke_width: 1,
    };
    for poly in obstacles.static_obstacles_with_offset.iter() {
        let drawing_poly: Vec<_> = poly
            .exterior()
            .coords_iter()
            .map(|point| (point.x as i32, point.y as i32))
            .collect();
        // println!("{drawing_poly:?}");
        let polygon = PathElement::new(drawing_poly, offset_polygon_color);
        root.draw(&polygon)?;
    }
    // Draw selected path with black color and point indexes
    let line_color = ShapeStyle {
        color: BLACK.to_rgba(),
        filled: false,
        stroke_width: 1,
    };
    let drawing_line: Vec<_> = path
        .iter()
        .map(|point| (point.x as i32, point.y as i32))
        .collect();
    let line = PathElement::new(drawing_line.clone(), line_color);
    drawing_line.iter().fold(0, |i, coord| {
        root.draw(&Text::new(
            i.to_string(),
            (coord.0, coord.1 + 8),
            ("sans-serif", 12.).into_font(),
        ))
        .expect("err i guess?");
        i + 1
    });
    root.draw(&line)?;

    Ok(())
}

fn build_visibility_graph_from_polygons(
    polygons: &MultiPolygon,
    polygons_with_offset: &MultiPolygon,
) -> Result<UnGraph<Coordinate, f64, usize>> {
    // let (mut sender_outer, mut receiver) = channel();
    let mut added_nodes: Vec<Coordinate> = Vec::with_capacity(polygons_with_offset.coords_count());
    let mut graph: UnGraph<Coordinate, f64, usize> = UnGraph::default();
    let mut edge_vec: Vec<(Coordinate, Coordinate)> = Vec::with_capacity(1024);

    for c1 in polygons_with_offset.exterior_coords_iter() {
        for c2 in polygons_with_offset.exterior_coords_iter() {      
            if c1 == c2 {
                continue;
            }
            if edge_vec.contains(&(c1, c2)) || edge_vec.contains(&(c2, c1)) {
                continue;
            }
            let line = Line::new(c1, c2);
            if !polygons.intersects(&line) {
                edge_vec.push((c1, c2));
                if !added_nodes.contains(&c1){ added_nodes.push(c1) }
                if !added_nodes.contains(&c2){ added_nodes.push(c2) }
            }
        }
    }
    
    added_nodes.iter().for_each(|node| {graph.add_node(*node);});

    for edge in edge_vec.iter() {
        graph.add_edge(
            added_nodes.iter().position(|x| *x == edge.0).unwrap().into(),
            added_nodes.iter().position(|x| *x == edge.1).unwrap().into(),
            Line::new(edge.0, edge.1).euclidean_length(),
        );
    }
    // println!("temp_vec len = {}", temp_vec.len());
    // let path = petgraph::algo::astar::astar(
    //     &graph,
    //     0.into(),
    //     |finish| finish == 11.into(),
    //     |e| *e.weight(),
    //     |_| 0.,
    // ).unwrap().1;
    // println!("graph:\n{:#?}", graph);
    // println!("path: {path:#?}");
    // debug_draw_visibility_graph(polygons_with_offset, edge_vec)?;

    Ok(graph)
}

fn debug_draw_visibility_graph(
    polygons: &MultiPolygon,
    lines: Vec<(Coordinate, Coordinate)>,
    // graph: UnGraph<Coordinate, f64, usize>,
) -> Result<()> {
    use plotters::prelude::Polygon;
    let root = BitMapBackend::new("debug_graph2.png", (1000, 1000)).into_drawing_area();
    root.fill(&WHITE)?;
    // Draw static obstacles with dark green color
    let polygon_color = ShapeStyle {
        color: GREEN_700.to_rgba().mix(0.7),
        filled: true,
        stroke_width: 2,
    };
    for poly in polygons {
        let drawing_poly: Vec<_> = poly
            .exterior()
            .coords_iter()
            .map(|point| (point.x as i32, point.y as i32))
            .collect();
        // println!("{drawing_poly:?}");
        let polygon = Polygon::new(drawing_poly, polygon_color);
        root.draw(&polygon)?;
    }
    let line_color = ShapeStyle {
        color: BLACK.to_rgba(),
        filled: false,
        stroke_width: 1,
    };
    for line in lines {
        let path = PathElement::new(
            [
                (line.0.x as i32, line.0.y as i32),
                (line.1.x as i32, line.1.y as i32),
            ],
            line_color,
        );
        root.draw(&path)?;
    }
    Ok(())
}
