use std::{fs::File, io::{BufReader, BufWriter}, time::Instant};

use anyhow::Result;
use geo_clipper::Clipper;
use petgraph::graph::UnGraph;
use plotters::{
    prelude::*,
    style::full_palette::{BLACK, GREEN_700},
};
mod ga;
use ga::{Enviroment, Obstacles, SelectionMethod, CrossoverParentSelection, CrossoverMethod, GeneticAlgorithm};
// use ga::GeneticAlgorithm;
use geo::{
    coord, Coordinate, CoordsIter, EuclideanLength, Intersects, Line, LineString, MultiPolygon,
    Polygon,
};

use serde::{Deserialize, Serialize};

#[cfg(test)]
mod tests;

#[derive(Debug, Serialize, Deserialize)]
pub struct Config {
    population_size: usize,
    generation_max: usize,
    selection_method: SelectionMethod,
    crossover_parent_selection: CrossoverParentSelection,
    crossover_method: CrossoverMethod,
    crossover_probability: f64,
    mutation_probability: f64,
    terminate_value: f64,
    elitism: bool,
}

fn main() -> Result<()> {
    let (obstacles, enviroment) = read_enviroment_from_file("env2.json").unwrap();
    draw_env_to_file("debug_env.png", &obstacles, &Vec::default()).unwrap();
    let file = File::open("config.json")?;
    let reader = BufReader::new(file);
    let config: Config = serde_json::from_reader(reader).unwrap();

    let mut ga = GeneticAlgorithm::new(obstacles, enviroment, config);
    println!("{:#?}", ga.config);
    let start = Instant::now();
    while !ga.terminate(){
        ga.step();
        if ga.generation % 25 == 0 { println!("Generation: {}", ga.generation) };
    }
    let elapsed = start.elapsed();
    
    draw_env_to_file("ga_results.png", &ga.obstacles, &ga.population.first().unwrap().points).unwrap();
    let statistics_json_file = File::options().write(true).create(true).truncate(true).open("simulation_statistics.json")?;
    let writer = BufWriter::new(statistics_json_file);
    serde_json::to_writer(writer, &ga.ga_statistics)?;
    println!("\nWeights:");
    ga.ga_statistics.last().unwrap().mutation_operators_weights.iter().zip(GeneticAlgorithm::OPERATOR_NAMES.iter()).for_each(|x|{
        println!("{}: {:.2}",x.1, x.0);
    });
    println!("Generations: {}", ga.generation);
    println!("Simulation time: {:.2?}", elapsed);

    Ok(())
}

fn read_enviroment_from_file(filename: &str) -> Result<(Obstacles, Enviroment)> {
    let file = File::open(filename)?;
    let reader = BufReader::new(file);
    let parsed_json: serde_json::Value = serde_json::from_reader(reader)?;
    let width = parsed_json["width"].as_f64().unwrap_or_default();
    let height = parsed_json["height"].as_f64().unwrap_or_default();
    let starting_point: Coordinate = serde_json::from_value(parsed_json["starting_point"].clone()).unwrap();
    let ending_point: Coordinate = serde_json::from_value(parsed_json["ending_point"].clone()).unwrap();
    let env = Enviroment {
        width: 1000.,
        height: 1000.,
        starting_point: (starting_point.x * (1000. / width), starting_point.y * (1000. / height)).into(),
        ending_point: (ending_point.x * (1000. / width), ending_point.y * (1000. / height)).into(),
    };
    let mut polygons = vec![];
    let array: Vec<Vec<Coordinate>> =
        serde_json::from_value(parsed_json["static_obstacles"].clone()).unwrap();
    for polygon in array.iter() {
        let test: LineString = LineString::new(polygon.iter().map(|coord| (coord.x * (1000. / width), coord.y * (1000. / height)).into()).collect());
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
        static_obstacles: multi_polygon,
        static_obstacles_with_offset: multi_polygon_with_offset,
        visibility_graph: visibility_graph,
    };
    Ok((obstacles, env))
}

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

// TODO: make this shit go fast with multithreading.
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
                if !added_nodes.contains(&c1) {
                    added_nodes.push(c1)
                }
                if !added_nodes.contains(&c2) {
                    added_nodes.push(c2)
                }
            }
        }
    }
    debug_draw_visibility_graph(polygons_with_offset, edge_vec.clone()).unwrap();

    added_nodes.iter().for_each(|node| {
        graph.add_node(*node);
    });

    for edge in edge_vec.iter() {
        graph.add_edge(
            added_nodes
                .iter()
                .position(|x| *x == edge.0)
                .unwrap()
                .into(),
            added_nodes
                .iter()
                .position(|x| *x == edge.1)
                .unwrap()
                .into(),
            Line::new(edge.0, edge.1).euclidean_length(),
        );
    }
    Ok(graph)
}

fn debug_draw_visibility_graph(
    polygons: &MultiPolygon,
    lines: Vec<(Coordinate, Coordinate)>,
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
