use rand::prelude::SliceRandom;
use rand::thread_rng;
use rand_distr::Distribution;
use rand_distr::Uniform;
use anyhow::anyhow;
use anyhow::Result;

use super::Individual;

pub fn roulette_selector(
    population: &Vec<Individual>,
    selection_count: usize,
) -> Result<Vec<Individual>> {
    let mut rng = thread_rng();
    let fitness_sum = population.iter().fold(0., |acc, ind| acc + ind.fitness);
    let mut probability_sum = 0.;
    let mut probability_vec: Vec<f64> = Vec::with_capacity(population.len());
    let mut result = Vec::with_capacity(selection_count);
    for individual in population.iter() {
        probability_vec.push(individual.fitness / fitness_sum);
    }
    // population.iter().for_each(|x| println!("fitness: {}", x.fitness));
    let mut selection_wheel_probability: Vec<f64> = Vec::default();
    for probability in probability_vec.iter() {
        probability_sum += (1. - probability) / (population.len() - 1) as f64;
        selection_wheel_probability.push(probability_sum);
    }
    // selection_wheel_probability.iter().for_each(|x| println!("selection prob: {}", x));
    let between = Uniform::new(0., 1.);
    for _ in 0..selection_count {
        let roll = between.sample(&mut rng);
        for (idx, probability) in selection_wheel_probability.iter().enumerate() {
            if roll < *probability {
                result.push(population[idx].clone());
                break;
            }
        }
    }

    Ok(result)
}

/// Selects `selection_count` individuals from old population in `selection_count / 2` tournaments consisting
/// of `participants_count` participants
pub fn tournament_selector(
    population: &Vec<Individual>,
    selection_count: usize,
    participants_count: usize,
) -> Result<Vec<Individual>> {
    if selection_count == 0 || selection_count % 2 != 0 || selection_count > population.len() {
        return Err(anyhow!("Invalid parameter `selection_count`: {}. Should be higher than 0, less than population size and a multiple of two.", selection_count));
    }
    if participants_count < 2 || participants_count > population.len() {
        return Err(anyhow!("Invalid parameter `participants_count`: {}. Should be higher than 2 and less than population size.", participants_count));
    }
    let mut result: Vec<Individual> = Vec::with_capacity(selection_count);
    let mut rng = thread_rng();

    for _ in 0..(selection_count / 2) {
        let mut tournament: Vec<Individual> = population
            .choose_multiple(&mut rng, participants_count)
            .cloned()
            .collect();
        tournament.sort_by(|a, b| a.fitness.total_cmp(&b.fitness));
        result.extend_from_slice(&tournament[0..2]);
    }
    Ok(result)
}

pub fn uniform_selector(
    population: &Vec<Individual>,
    selection_count: usize,
) -> Result<Vec<Individual>> {
    let mut rng = thread_rng();
    let mut result = Vec::with_capacity(selection_count);
    let indexes = rand::seq::index::sample(&mut rng, population.len(), selection_count);
    for idx in indexes {
        result.push(population[idx].clone());
    }
    Ok(result)
}

pub fn stochastic_universal_sampling_selector(
    population: &Vec<Individual>,
    selection_count: usize,
) -> Result<Vec<Individual>> {
    let mut minimize_fitness_vec: Vec<f64> = Vec::with_capacity(population.len());
    let max_fitness: f64 = population
        .iter()
        .max_by(|x, y| x.fitness.total_cmp(&y.fitness))
        .unwrap()
        .fitness;
    let bias = max_fitness / 6.;
    for ind in population.iter() {
        minimize_fitness_vec.push(max_fitness - ind.fitness + bias);
        // println!("minimize_fitness: {}", max_fitness - ind.fitness + bias);
    }
    let minimize_fitness_sum: f64 = minimize_fitness_vec.iter().sum();
    let mut current_point = minimize_fitness_vec.first().unwrap().clone();
    let chunk = minimize_fitness_sum / selection_count as f64;
    let mut idx = 0;
    let mut result = Vec::with_capacity(selection_count);
    // println!("chunk: {chunk:.4}, minimize_fitness_sum: {minimize_fitness_sum}");
    for i in 0..selection_count {
        'inner: loop {
            if (i as f64 + 1.) * chunk <= (current_point + 0.01) {
                result.push(population[idx].clone());
                break 'inner;
            } else {
                idx += 1;
                current_point += minimize_fitness_vec[idx];
            }
        }
    }
    Ok(result)
}