use std::{fs::File, io::BufReader};
use anyhow::Result;
use geo::{line_string, Coordinate};
use rayon::slice::ParallelSliceMut;
use crate::{
    ga::{Individual, selection::{stochastic_universal_sampling_selector, roulette_selector}, GeneticAlgorithm, mutation::{shorten_path_mutate, repair_mutation}},
    read_enviroment_from_file, Config,
};

#[test]
fn test_ga() -> Result<()>{
    let (obstacles, enviroment) = read_enviroment_from_file("test_env.json")?;
    println!("{:?}, {:?}", enviroment.starting_point, enviroment.ending_point);
    let file = File::open("config.json")?;
    let reader = BufReader::new(file);
    let config: Config = serde_json::from_reader(reader).unwrap();

    let pop_size = config.population_size;
    let ga = GeneticAlgorithm::new(obstacles, enviroment, config);
    assert_eq!(ga.population.len(), pop_size);

    Ok(())
}


#[test]
fn test_shorten_mutate() -> Result<()>{
    let (obstacles, enviroment) = read_enviroment_from_file("test_env.json")?;
    let file = File::open("config.json")?;
    let reader = BufReader::new(file);
    let config: Config = serde_json::from_reader(reader).unwrap();
    let ga = GeneticAlgorithm::new(obstacles, enviroment, config);
    
    let test_path = line_string![
        (180., 950.).into(),
        (460., 850.).into(),
        (730., 580.).into(),
        (680., 330.).into()
    ];
    let mut temp_individual = Individual::new(test_path.0, vec![1.;10]);
    shorten_path_mutate(&ga, &mut temp_individual);
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
    let mut temp_individual = Individual::new(test_path.0, vec![1.;10]);
    shorten_path_mutate(&ga, &mut temp_individual);
    assert_eq!(
        temp_individual.points,
        &[(180., 950.).into(), (880., 375.).into()]
    );
    Ok(())
}

#[test]
fn test_repair_mutate() -> Result<()> {
    let (obstacles, enviroment) = read_enviroment_from_file("test_env.json")?;
    let file = File::open("config.json")?;
    let reader = BufReader::new(file);
    let config: Config = serde_json::from_reader(reader).unwrap();

    let ga = GeneticAlgorithm::new(obstacles, enviroment, config);
    let test_path = line_string![
        (100., 820.).into(),
        (920., 840.).into(),
        (900., 500.).into(),
    ];
    let mut temp_individual = Individual::new(test_path.0, vec![1.;10]);
    repair_mutation(&ga, &mut temp_individual);
    let expected = Individual {
        fitness: 10000000000.0,
        feasible: false,
        dynamic_feasible: false,
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
        evaluated: false,
        speed: vec![],
    };
    assert_eq!(temp_individual, expected);
    Ok(())
}

#[test]
#[rustfmt::skip]
fn test_stochastic_universal_sampling_selector(){
    let mut test_pop: Vec<Individual> = Vec::new();
    test_pop.push(Individual{ fitness: 50.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 30.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 80.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 110., feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 10.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 70.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 85.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 90.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 2.,   feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 12.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 15.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.par_sort_by(|a,b| a.fitness.total_cmp(&b.fitness));
    let result = stochastic_universal_sampling_selector(&test_pop, test_pop.len()).unwrap();
    let expected = [
        Individual {
            fitness: 2.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 10.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 10.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 12.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 15.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 15.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 30.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 50.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 70.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 85.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
        Individual {
            fitness: 110.0,
            feasible: false,
            dynamic_feasible: false,
            points: vec![],
            evaluated: false,
            speed: vec![],
        },
    ];
    assert_eq!(result, expected);
}

#[test]
#[rustfmt::skip]
fn test_roulette_selector(){
    let mut test_pop: Vec<Individual> = Vec::new();
    test_pop.push(Individual{ fitness: 50.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 30.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 80.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 110., feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 10.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 70.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 85.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 90.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 2.,   feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 12.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.push(Individual{ fitness: 15.,  feasible: false, dynamic_feasible: false, points:  vec![], evaluated: false, speed: vec![],});
    test_pop.par_sort_by(|a,b| a.fitness.total_cmp(&b.fitness));
    let result = roulette_selector(&test_pop, test_pop.len()).unwrap();
    println!("{result:?}");
}
