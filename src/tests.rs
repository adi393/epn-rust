use anyhow::Result;
use geo::{line_string, MultiPolygon, Coordinate};
use petgraph::graph::UnGraph;
use wkt::TryFromWkt;

use crate::{
    build_visibility_graph_from_polygons, draw_env_to_file,
    ga::{repair_mutation, shorten_path_mutate, Individual, Obstacles, stochastic_universal_sampling_selector, roulette_selector},
    read_enviroment_from_file,
};

#[test]
fn test_shorten_mutate() {
    let multi_polygon: MultiPolygon<f64> = MultiPolygon::try_from_wkt_str("MULTIPOLYGON(((50 30,230 120,40 200,50 30)),((620 60,920 70,920 220,580 280,620 60)),((800 600,960 720,820 900,630 800,800 600)),((100 250,400 260,580 340,680 530,290 850,110 650,480 540,100 470,100 250)))").unwrap();
    let obstacles = Obstacles {
        static_obstacles: multi_polygon.clone(),
        static_obstacles_with_offset: multi_polygon.clone(),
        visibility_graph: UnGraph::default(),
    };
    let test_path = line_string![
        (180., 950.).into(),
        (460., 850.).into(),
        (730., 580.).into(),
        (680., 330.).into()
    ];
    let mut temp_individual = Individual::new(test_path.0);
    shorten_path_mutate(&mut temp_individual, &obstacles);
    assert_eq!(
        temp_individual.points,
        &[
            (180., 950.).into(),
            (730., 580.).into(),
            (680., 330.).into()
        ]
    );

    let test_path = line_string![
        (180., 950.).into(),
        (460., 850.).into(),
        (730., 580.).into(),
        (880., 375.).into()
    ];
    let mut temp_individual = Individual::new(test_path.0);
    shorten_path_mutate(&mut temp_individual, &obstacles);
    assert_eq!(
        temp_individual.points,
        &[(180., 950.).into(), (880., 375.).into()]
    );
}

#[test]
fn test_repair_mutate() -> Result<()> {
    let (obstacles, _enviroment) = read_enviroment_from_file("env.json")?;
    let test_path = line_string![
        (100., 820.).into(),
        (920., 840.).into(),
        (900., 500.).into(),
    ];
    let mut temp_individual = Individual::new(test_path.0);
    repair_mutation(&mut temp_individual, &obstacles);
    let expected = Individual {
        fitness: 10000000000.0,
        feasible: false,
        points: vec![
            Coordinate {
                x: 100.0,
                y: 820.0,
            },
            Coordinate {
                x: 290.0,
                y: 854.0,
            },
            Coordinate {
                x: 821.0,
                y: 904.0,
            },
            Coordinate {
                x: 920.0,
                y: 840.0,
            },
            Coordinate {
                x: 900.0,
                y: 500.0,
            },
        ],
    };
    assert_eq!(temp_individual, expected);
    Ok(())
}

#[test]
#[rustfmt::skip]
fn test_stochastic_universal_sampling_selector(){
    let mut test_pop: Vec<Individual> = Vec::new();
    test_pop.push(Individual{ fitness: 50., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 30., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 80., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 110., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 10., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 70., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 85., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 90., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 2., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 12., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 15., feasible: false, points: vec![] });
    test_pop.sort_by(|a,b| a.fitness.total_cmp(&b.fitness));
    let result = stochastic_universal_sampling_selector(&test_pop, test_pop.len()).unwrap();
    let expected = [
        Individual {
            fitness: 2.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 10.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 10.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 12.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 15.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 15.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 30.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 50.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 70.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 85.0,
            feasible: false,
            points: vec![],
        },
        Individual {
            fitness: 110.0,
            feasible: false,
            points: vec![],
        },
    ];
    assert_eq!(result, expected);
}

#[test]
#[rustfmt::skip]
fn test_roulette_selector(){
    let mut test_pop: Vec<Individual> = Vec::new();
    test_pop.push(Individual{ fitness: 50., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 30., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 80., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 110., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 10., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 70., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 85., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 90., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 2., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 12., feasible: false, points: vec![] });
    test_pop.push(Individual{ fitness: 15., feasible: false, points: vec![] });
    test_pop.sort_by(|a,b| a.fitness.total_cmp(&b.fitness));
    let result = roulette_selector(&test_pop, test_pop.len()).unwrap();
    println!("{result:?}");
}
