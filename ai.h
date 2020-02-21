/**********************************************
* Header file for ai.c. Written by Tom Hanson *
* 913022 - (2019)                          *   
***********************************************/

#ifndef __AI__
#define __AI__

#include <stdint.h>
#include <unistd.h>
#include "node.h"
#include "priority_queue.h"

struct output {
    propagation_t prop;
    int budget;
    int max_depth;
    int generated;
    int expanded;
    float node_sec;
    float time;
    int max_board;
    int score;
};

typedef struct output output_t;


struct heap* initialize_ai(void);
node_t* create_init_node( state_t* init_state,move_t prev);
void copy_state(state_t* dst, state_t* src);

move_t get_next_move( state_t init_state, int budget, propagation_t propagation, char* stats,move_t prev,move_t p_prev, output_t* output);
void exploreAction(node_t* current_node,node_t** new_node,node_t** explored,int* i,propagation_t propagation,int budget,move_t move,move_t p_prev, output_t* output);
move_t getBestAction(node_t* explored[],float best_action_score[]);

bool lostLife(node_t* n);
bool checkDir(state_t state,move_t move);
bool checkBacktrack(node_t* node,move_t action);
void propagateBackScoreToFirstAction(node_t* new_node,propagation_t propagation);
void testAction(node_t* n, node_t* new_node, move_t action );
bool applyAction(node_t* n, node_t** new_node, move_t action );
float heuristic( node_t* n );
float get_reward ( node_t* n );
move_t getOpposite(move_t action);


#endif
