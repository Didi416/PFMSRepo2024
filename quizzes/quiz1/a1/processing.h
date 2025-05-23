#ifndef PROCESSING_H
#define PROCESSING_H
#include "person.h"
#include <vector>

/**
 * @brief Returns the `Person` who is the oldest person(s) of the `crowd`.
 * @param crowd vector of Perso
 * @return The vector of oldest people, if multiple people are same age (oldest) they all need to be returned in same order as original vector.
 */
std::vector<Person> oldestPerson(std::vector<Person> crowd);


/**
 * @brief Function that returns person(s) that need to be vaccinated 
 * older or equal to the specified `ageCuttoff` and have not been previously vaccinated
 * @param crowd vector of Person
 * @param ageCutoff the age which people are eligible (for example 18+ means 18 and older)
 * @return The vector of people eligible for vaccination, needs to be organised in the order oldest to youngest (the oldest is at the begining and youngest and end of vetor).
 */
std::vector<Person> eligibleForVaccine(std::vector<Person> crowd, unsigned int ageCutoff);


#endif // PROCESSING_H
