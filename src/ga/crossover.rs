use std::{sync::mpsc::channel};
use geo::{Coordinate};
use rand::{random, thread_rng, seq::index::sample, Rng};
use rand_distr::{Uniform, Distribution};
use rayon::iter::{ParallelBridge, ParallelIterator};


use super::{CrossoverMethod, Individual, GeneticAlgorithm};

pub fn crossover_opposite_ends_parents(ga: &mut GeneticAlgorithm, new_population: &mut Vec<Individual>) {
    let (sender, receiver) = flume::unbounded();
    let len = new_population.len();
    let (left, right) = new_population.split_at_mut(len / 2);
    let zipped = left.iter_mut().zip(right.iter_mut().rev());
    zipped.par_bridge().for_each_with(sender, |s,(parent1, parent2)|{
        // Roll the probability
        let roll: f64 = random();
        if roll > ga.config.crossover_probability { return; }
        s.send(()).unwrap();
        let (child1, child2) = match ga.config.crossover_method {
            CrossoverMethod::TwoPoint => two_point_crossover(parent1, parent2),
            CrossoverMethod::Blend { alpha } => todo!(),
            CrossoverMethod::CoinToss => todo!(),
            CrossoverMethod::Shuffle => todo!(),
        };
        *parent1 = child1;
        *parent2 = child2;
        parent1.evaluated = false;
        parent2.evaluated = false;
    });
    let operator_calls: Vec<()> = receiver.iter().collect();
    ga.current_generation_stats.crossover_operators_uses = operator_calls.len();
}

pub fn crossover_sequential_parents(ga: &mut GeneticAlgorithm, new_population: &mut Vec<Individual>) {
    //println!("crossover_sequential_parents");
    let (sender, receiver) = flume::unbounded();
    new_population.chunks_mut(2).par_bridge().for_each_with(sender, |s,chunk| {
        // Roll the probability
        let roll: f64 = random();
        if roll > ga.config.crossover_probability { return; }
        s.send(()).unwrap();
        if chunk.len() != 2 {
            return;
        }
        let (child1, child2) = match ga.config.crossover_method {
            CrossoverMethod::TwoPoint => two_point_crossover(&chunk[0], &chunk[1]),
            CrossoverMethod::Blend { alpha } => todo!(),
            CrossoverMethod::CoinToss => todo!(),
            CrossoverMethod::Shuffle => todo!(),
        };
        chunk[0] = child1;
        chunk[1] = child2;
        chunk[0].evaluated = false;
        chunk[1].evaluated = false;
    });
    let operator_calls: Vec<()> = receiver.iter().collect();
    ga.current_generation_stats.crossover_operators_uses = operator_calls.len();
    

}

pub fn crossover_random_parents(ga: &mut GeneticAlgorithm, new_population: &mut Vec<Individual>) {
   // println!("crossover_random_parents");
    let mut crossover_uses = 0usize;
    let mut rng = thread_rng();
    for _ in 0..(ga.population.len() / 2) {
        // Roll the probability
        let roll = rng.gen_range(0. ..1.);
        if roll > ga.config.crossover_probability { continue; }
        crossover_uses += 1;
        let indices = sample(&mut rng, ga.population.len(), 2).into_vec();
        let (child1, child2) = match ga.config.crossover_method {
            CrossoverMethod::TwoPoint => {
                two_point_crossover(&ga.population[indices[0]], &ga.population[indices[1]])
            }
            CrossoverMethod::Blend { alpha } => todo!(),
            CrossoverMethod::CoinToss => todo!(),
            CrossoverMethod::Shuffle => todo!(),
        };
        new_population[indices[0]] = child1;
        new_population[indices[1]] = child2;
        new_population[indices[0]].evaluated = false;
        new_population[indices[1]].evaluated = false;
    }
    ga.current_generation_stats.crossover_operators_uses = crossover_uses;
}

pub fn two_point_crossover(
    individual1: &Individual,
    individual2: &Individual,
) -> (Individual, Individual) {
    let mut rng = thread_rng();
    let lengths = [individual1.points.len(), individual2.points.len()];
    let min_len = lengths.iter().min().unwrap();
    if *min_len <= 2usize { return (individual1.clone(), individual2.clone()); }
    let range = Uniform::new(1, min_len - 1);


    let c: Vec<_> = range.sample_iter(&mut rng).take(2).collect();
    let crossover_low = c.iter().min().unwrap();
    let crossover_high = c.iter().max().unwrap();

    let mut result1 = Vec::<Coordinate>::with_capacity(*crossover_high);
    let mut result2 = Vec::<Coordinate>::with_capacity(*crossover_high);
    
    // Copy the part before crossover
    result1.extend_from_slice(&individual1.points[0..*crossover_low]);
    result2.extend_from_slice(&individual2.points[0..*crossover_low]);

    // Copy the crossover part from opposite individuals
    result1.extend_from_slice(&individual2.points[*crossover_low..*crossover_high]);
    result2.extend_from_slice(&individual1.points[*crossover_low..*crossover_high]);

    // Copy the remaining points.
    result1.extend_from_slice(&individual1.points[*crossover_high..]);
    result2.extend_from_slice(&individual2.points[*crossover_high..]);

    (Individual::new(result1), Individual::new(result2))
}