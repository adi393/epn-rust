#[allow(unused_imports)]
use anyhow::anyhow;

#[allow(unused_imports)]
use anyhow::Result;
use geo::{coord, Coordinate, EuclideanLength, Intersects, MultiLineString, MultiPolygon, Polygon};
use geo_clipper::ClipperOpen;
#[allow(unused_imports)]
use petgraph::algo::astar;
use petgraph::graph::UnGraph;
use rand::{distributions::Uniform, random, thread_rng, Rng};
use rand_distr::Distribution;
#[allow(unused_imports)]
use rayon::{
    iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator},
    vec,
};
use serde::{Deserialize, Serialize};
use std::{
    f64::consts::PI,
    sync::mpsc::{channel, Receiver, Sender},
};

use crate::Config;

use self::{
    crossover::{
        crossover_opposite_ends_parents, crossover_random_parents, crossover_sequential_parents,
    },
    mutation::{
        delete_mutation, hard_mutation, insert_if_invalid_mutation, move_mutation, repair_mutation,
        shorten_path_mutate, swap_mutation,
    },
    selection::{
        roulette_selector, stochastic_universal_sampling_selector, tournament_selector,
        uniform_selector,
    },
};

pub mod crossover;
pub mod mutation;
pub mod selection;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Individual {
    pub fitness: f64,
    pub feasible: bool,
    pub points: Vec<Coordinate<f64>>,
    pub evaluated: bool,
}

impl Individual {
    pub fn new(points: Vec<Coordinate<f64>>) -> Individual {
        Individual {
            fitness: 1e10,
            feasible: false,
            points,
            evaluated: false,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GenerationStatistic {
    generation: usize,
    population: Vec<Individual>,
    mutation_operators_weights: Vec<f64>,
    mutation_operators_uses: Vec<usize>,
    crossover_operators_uses: usize,
}

impl GenerationStatistic {
    pub fn new(mutation_operators: usize) -> Self {
        Self {
            generation: 0,
            population: vec![],
            mutation_operators_weights: vec![1.; mutation_operators],
            mutation_operators_uses: vec![0; mutation_operators],
            crossover_operators_uses: 0,
        }
    }
}

pub struct GeneticAlgorithm {
    pub config: Config,
    pub enviroment: Enviroment,
    pub generation: usize,
    pub mutation_operators_weights: Vec<f64>,
    pub mutation_operators: Vec<&'static (dyn Fn(&GeneticAlgorithm, &mut Individual) + Sync)>,
    pub obstacles: Obstacles,
    pub population: Vec<Individual>,
    pub ga_statistics: Vec<GenerationStatistic>,
    pub current_generation_stats: GenerationStatistic,
}
impl GeneticAlgorithm {
    pub fn new(obstacles: Obstacles, enviroment: Enviroment, config: Config) -> Self {
        let population = GeneticAlgorithm::initialize_population(&enviroment, &config);
        let operators: Vec<&'static (dyn Fn(&GeneticAlgorithm, &mut Individual) + Sync)> = vec![
            &hard_mutation,
            &swap_mutation,
            &move_mutation,
            &delete_mutation,
            &insert_if_invalid_mutation,
            &shorten_path_mutate,
            &repair_mutation,
        ];
        let mutation_operators_weights = vec![1.; operators.len()];
        let current_generation_stats = GenerationStatistic::new(operators.len());
        Self {
            obstacles,
            enviroment,
            population,
            config,
            generation: 0,
            mutation_operators_weights,
            mutation_operators: operators,
            ga_statistics: vec![],
            current_generation_stats,
        }
    }

    fn initialize_population(enviroment: &Enviroment, config: &Config) -> Vec<Individual> {
        let mut vec = Vec::with_capacity(config.population_size);
        let between_width = Uniform::new(0., enviroment.width);
        let between_height = Uniform::new(0., enviroment.height);
        let mut rng = rand::thread_rng();
        for _ in 0..config.population_size {
            let mut points = Vec::with_capacity(10);
            points.push(enviroment.starting_point);
            for _ in 0..8 {
                points.push(
                    coord! {x: between_width.sample(&mut rng), y: between_height.sample(&mut rng)},
                );
            }
            points.push(enviroment.ending_point);
            vec.push(Individual::new(points));
        }
        vec
    }

    fn evaluate(&self, individual: &mut Individual) {
        // println!("evaluate");
        let path = MultiLineString::new(vec![individual.points.clone().into()]);
        let path_length = path.euclidean_length();
        let test = &self.obstacles.static_obstacles.0;
        let (sender, receiver): (Sender<f64>, Receiver<f64>) = channel();
        test.par_iter().for_each_with(sender, |sender, poly| {
            if poly.intersects(&path) {
                let intersection = path.intersection(poly, 1.);
                sender.send(intersection.euclidean_length()).unwrap();
            }
        });
        let mut angle_cost = 0.;

        for slice in individual.points.windows(3) {
            if let [p1, p2, p3] = slice {
                let angle1 = (p2.y - p1.y).atan2(p2.x - p1.x);
                let angle2 = (p2.y - p3.y).atan2(p2.x - p3.x);
                let result = {
                    let mut x = (angle2 - angle1) * 180. / PI;
                    if x < 0. {
                        x += 360.;
                    }
                    if x > 180. {
                        x = 360. - x;
                    }
                    x
                };
                // println!("Line angle: {result}");
                angle_cost += (180. - result).powf(2.);
            }
        }

        let length_in_objects: f64 = receiver.iter().sum();
        if length_in_objects > 0. {
            individual.feasible = false;
        } else {
            individual.feasible = true;
        }
        let x = path_length + length_in_objects * 100. + angle_cost;
        individual.fitness = x;
        individual.evaluated = true;
    }

    pub fn step(&mut self) {
        // println!("step");
        // let population_range = Uniform::new(0, self.population.len());
        let mut new_population = self.population.clone();

        match self.config.crossover_parent_selection {
            CrossoverParentSelection::Random => crossover_random_parents(self, &mut new_population),
            CrossoverParentSelection::Sequential => {
                crossover_sequential_parents(self, &mut new_population)
            }
            CrossoverParentSelection::OppositeEnds => {
                crossover_opposite_ends_parents(self, &mut new_population)
            }
        }

        new_population
            .par_iter_mut()
            .for_each(|individual| self.evaluate(individual));
        // let (s, receiver) = channel();
        let mut mutation_stats: Vec<_> = Vec::default();
        for individual in new_population.iter_mut() {
            if individual.points.len() <= 2 {
                continue;
            }
            let mut rng = thread_rng();
            let roll: f64 = random();
            if roll < self.config.mutation_probability {
                let operator_idx = rng.gen_range(0..self.mutation_operators_weights.len());

                let fitness_before = individual.fitness;
                self.mutation_operators[operator_idx](self, individual);
                self.evaluate(individual);
                let fitness_after = individual.fitness;
                mutation_stats.push((
                    operator_idx,
                    (fitness_before - fitness_after) / fitness_before,
                ));
            }
        }

        // Update mutation weights
        for stat in mutation_stats.iter() {
            // let x = match stat.1 {
            //     x if x > 0. => -f64::log2(1. - x) / 100.,
            //     x if x < 0. => -f64::log2(1. + x.abs()) / 100.,
            //     _ => 0.,
            // };
            let x = 0.;
            self.mutation_operators_weights[stat.0] += x;
            self.current_generation_stats.mutation_operators_uses[stat.0] += 1;
        }
        self.population = match self.config.selection_method {
            SelectionMethod::Tournament {
                selection_count,
                participants_count,
            } => tournament_selector(&new_population, selection_count, participants_count).unwrap(),
            SelectionMethod::Uniform { selection_count } => {
                uniform_selector(&new_population, selection_count).unwrap()
            }
            SelectionMethod::StochasticUniversalSampling { selection_count } => {
                stochastic_universal_sampling_selector(&new_population, selection_count).unwrap()
            }
            SelectionMethod::Roulette { selection_count } => {
                roulette_selector(&new_population, selection_count).unwrap()
            }
        };

        self.population
            .sort_by(|a, b| a.fitness.total_cmp(&b.fitness));
        self.generation += 1;
        self.current_generation_stats.mutation_operators_weights =
            self.mutation_operators_weights.clone();
        self.current_generation_stats.generation = self.generation;
        self.current_generation_stats.population = self.population.clone();
        self.ga_statistics
            .push(self.current_generation_stats.clone());
        self.current_generation_stats = GenerationStatistic::new(self.mutation_operators.len());
    }

    pub fn terminate(&self) -> bool {
        // println!("terminate");
        if self.generation >= 25 {
            let current_fitness = self.population.first().unwrap().fitness;
            let fitness_10_before = self
                .ga_statistics
                .iter()
                .rev()
                .skip(9)
                .next()
                .unwrap()
                .population
                .first()
                .unwrap()
                .fitness;

            if self.population.first().unwrap().feasible
                && ((current_fitness - fitness_10_before).abs() <= self.config.terminate_value)
            {
                return true;
            }
        }

        if self.generation >= self.config.generation_max {
            return true;
        }
        return false;
    }
}
#[derive(Debug, Clone)]
pub struct Obstacles {
    pub static_obstacles: MultiPolygon<f64>,
    pub static_obstacles_with_offset: MultiPolygon<f64>,
    // pub dynamic_obstacles: Vec::<DynamicObstacle>,
    pub visibility_graph: UnGraph<Coordinate, f64, usize>,
}
#[derive(Debug, Clone)]
pub struct DynamicObstacle {
    pub polygon: Polygon<f64>,
    pub direction_angle: f64,
    pub speed: f64,
}
#[derive(Debug, Clone)]
pub struct Enviroment {
    pub width: f64,
    pub height: f64,
    pub starting_point: Coordinate,
    pub ending_point: Coordinate,
}

// TODO: Blend Crossover
// TODO: Shuffle crossover
// TODO: CoinToss crossover

// TODO: Rank selection

#[derive(Debug, Serialize, Deserialize)]
pub enum SelectionMethod {
    Tournament {
        selection_count: usize,
        participants_count: usize,
    },
    Uniform {
        selection_count: usize,
    },
    StochasticUniversalSampling {
        selection_count: usize,
    },
    Roulette {
        selection_count: usize,
    },
}

#[derive(Debug, Serialize, Deserialize)]
pub enum CrossoverParentSelection {
    Random,
    Sequential,
    OppositeEnds,
}
#[derive(Debug, Serialize, Deserialize)]
pub enum CrossoverMethod {
    TwoPoint,
    Blend { alpha: f64 },
    CoinToss,
    Shuffle,
}
