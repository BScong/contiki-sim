/*
 * Copyright (c) 2018, Beshr Al Nahas.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */

/**
 * \file
 *         Collect sensor data using NullNet
 *         Part of the assignment skeleton for the course Advanced Distributed Systems at Chalmers University of Technology
 * \author
 *        Beshr Al Nahas <beshr@chalmers.se>
 */

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "sys/energest.h"
#include "random.h"
#include "node-id.h"
#if MAC_CONF_WITH_TSCH
#include "net/mac/tsch/tsch.h"
#endif /* MAC_CONF_WITH_TSCH */
#include <string.h>
#include <stdio.h> /* For printf() */
#include "motes-xy-lookup-table.h"
/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "APP"
#define LOG_LEVEL LOG_LEVEL_INFO /* You can use LOG_LEVEL_DBG */
#define LOG_CONF_LEVEL_APP  LOG_LEVEL_INFO /* You can use LOG_LEVEL_DBG */
/*---------------------------------------------------------------------------*/


/* Types */
/* define the short address type as 16bits */
typedef uint16_t short_address_t;

/* The structure for the messages
 * Here you can add additional fields that you might need.
 */
typedef struct __attribute__((packed)) rout_msg {
  uint32_t seq;         /* packet sequence number */
  short_address_t from; /* Original sender.
                           We will not include the address of the receiver as we emply that all messages are directed to the sink. */
  uint16_t content;
  uint16_t hops;
  uint64_t bat;
  uint8_t type;         /* TYPE_ANNOUNCEMENT or TYPE_CONTENT*/
  double batt;
} rout_msg_t;

/* Message type identifier */
enum message_type_enum { TYPE_ANNOUNCEMENT = 11, TYPE_CONTENT };

/* define a boolean type for enhancing code readability */
typedef uint8_t bool;
#define TRUE 1
#define FALSE 0
/*---------------------------------------------------------------------------*/
/* Topology */
static const uint16_t motes_xy_lookup_table[][2] = MOTES_XY_LOOKUP_TABLE;

/* How long can the radio reach. You can change this in the simualtion settings, but then you need to change it here. */
#define RADIO_RANGE 50

/* The starting battery level */
/* Assume a battery capacity of 1mAh = 1000 microAh = 1000*3600*1000 mA/microseconds */
#define BATTERY_CAPACITY_MICRO_AH (100)
#define BATTERYSTART ((uint64_t)BATTERY_CAPACITY_MICRO_AH*3600*1000)

/* Radio energy usage *
 * Assume: 10mA for tx/rx/idle , 0 for off */
#define RADIO_TX_CURRENT 10u
#define RADIO_RX_CURRENT 10u
/*---------------------------------------------------------------------------*/
/* Configuration */
/* ID of the node that acts as the sink */
#define SINKNODE 1
#define SEND_INTERVAL (1 * CLOCK_SECOND)

/* Number of different rounds in protocol and the rounds themselves */
#define ROUNDS 2
/* Round type identifier */
enum round_type_enum { ROUND_ANNOUNCEMENT = 0, ROUND_CONTENT };

/* Whether the battery model should be used */
#define USEBATTERY 1

/* Whether basic routing should be used */
#define ROUTERTYPE 6
#define AGGREGATION 0
// ROUTERTYPE 0 : BASIC, 1: MIN HOPS, 2: MAX HOPS
// BASIC : around 1300 msgs (1321)
// MIN HOPS : around 1400 msgs (1433)
// MAX HOPS : around 1000 msgs (193)
/*---------------------------------------------------------------------------*/
/* default router address is the broadcast address: all zeros == linkaddr_null */
static linkaddr_t router_addr = {{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};
static const linkaddr_t sink_addr =   {{ SINKNODE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};
/*---------------------------------------------------------------------------*/
static uint32_t roundcounter = 0; /* Counting communication rounds so far */
static uint32_t local_hops = 0;
static uint64_t router_battery = 0;
static double   router_metric  = 0;
static int32_t local_distance = 0;
static int32_t router_distance = 0;
static uint64_t battery = BATTERYSTART; /* Battery capacity estimate */
static uint64_t current_battery = BATTERYSTART;
static struct etimer periodic_timer; /* Wakeup timer */
/*---------------------------------------------------------------------------*/
/* Helper functions */
/* get the message type as a string */
#define GET_MSG_TYPE_STR(T) ( (T) == TYPE_ANNOUNCEMENT ? "ANNOUNCEMENT" : ((T) == TYPE_CONTENT ? "CONTENT" : "UNKNOWN") )

/* Returns a random number between 0 and n-1 (both inclusive) */
uint16_t randommmm(uint16_t n) {
   uint16_t r = (uint16_t) random_rand();
   LOG_INFO("RANDOM: %u, n: %u, RANDOM MODULO N: %u", r, n, r % n);
   return (r % n);
}

/* Comuptes the square of the distance. We do not use the sqaure root function to have light-weight computations. */
int32_t distance_x_y(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by){
  return (int32_t)((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
}

int32_t distance_between(short_address_t aid, short_address_t bid){
  /* Use a lookup table */
  uint16_t ax = motes_xy_lookup_table[aid-1][0];
  uint16_t ay = motes_xy_lookup_table[aid-1][1];
  uint16_t bx = motes_xy_lookup_table[bid-1][0];
  uint16_t by = motes_xy_lookup_table[bid-1][1];
  return distance_x_y(ax, ay, bx, by);
}

int32_t distance_to_sink(short_address_t id){
  return distance_between(SINKNODE, id);
}

bool can_reach_sink(short_address_t node){
  return distance_to_sink(node) < RADIO_RANGE*RADIO_RANGE;
}

/*---------------------------------------------------------------------------*/
/* Routing helping functions */

bool is_sink() {
  return SINKNODE == node_id;
  /* Instead, we could test:
   * sink_addr.u16[0] == linkaddr_node_addr.u16[0];
   */
  /* No need to compare the whole address as we are only changing the first two bytes of the address anyway
   *  return linkaddr_cmp(&sink_addr, &linkaddr_node_addr);
   *
   */
}

bool has_router(){
  return (router_addr.u16[0] != 0);
  /* No need to compare the whole address as we are only changing the first two bytes of the address anyway
   *  return !linkaddr_cmp(&router_addr, &linkaddr_null);
   *
   */
}

static bool aggregator = 0;
static uint16_t agg_sum = 0;

void set_aggregator(bool agg){
   aggregator = agg;
}

bool is_aggregator(){
   return aggregator;
}

void set_router(short_address_t new_router_id){
  router_addr.u16[0] = new_router_id;
  LOG_INFO("Router: updated to %u\n", new_router_id);
}

void sink_collect_data(const rout_msg_t * msg){
  static uint32_t collected_so_far = 0;
  collected_so_far += msg->content;
  LOG_INFO("Sink: received %u from %u\n", msg->content, msg->from);
  LOG_INFO("Sink: collected so far %u\n", collected_so_far);
}

short_address_t neighbors[12];
double metrics[12];
uint16_t probs[12];

void update_metric(short_address_t node, double metric){
   uint8_t i;
   for(i = 0; i < 12; i++){
      if(neighbors[i] == node || neighbors[i] == 99){
	neighbors[i] = node;
 	metrics[i] = metric;
	break;
      }
   }

   double sum = 0;
   for(i = 0; i < 12 && neighbors[i] != 99; i++){
	sum += metrics[i];
   }

   for(i = 0; i < 12 && neighbors[i] != 99; i++){
	probs[i] = (uint16_t) (100 * (metrics[i]/sum));
        LOG_INFO("SET PROB TO %u \n", probs[i]);
   }
}

void select_router(){
   //LOG_INFO("SELECTION \n");
   short_address_t rout = neighbors[0];
   uint16_t prob        = probs[0];
   uint16_t rng         = randommmm(100);
   uint8_t  i;
   LOG_INFO("NODE SELECTION RNG: %u . RNG < 100: %u \n", rng, rng < 100);
   for(i = 0; i < 12; i++){
	if(rng <= probs[i] && prob < probs[i]){
	   prob = probs[i];
           rout = neighbors[i];
           router_metric = metrics[i];
        }
   }
   set_router(rout);
}


static uint16_t threshold = 50;
static uint16_t cmmbcr_mets[12][2];
void update_metrics_cmmbcr(short_address_t node, uint16_t batt, uint16_t hops){
   uint8_t i;
   for(i = 0; i < 12; i++){
      if(neighbors[i] == node || neighbors[i] == 99){
        neighbors[i] = node;
        cmmbcr_mets[i][0] = batt;
        cmmbcr_mets[i][1] = hops;
        break;
      }
   }

}


void select_router_cmmbcr(){
    if(can_reach_sink(node_id)){
       set_router(SINKNODE);
    }else{
      uint8_t i;
      static uint16_t b = 0;
      uint8_t best_rout;
      for(i = 0; i < 12 && neighbors[i] != 99; i++){
	  if(cmmbcr_mets[i][0] > b){
	     b = cmmbcr_mets[i][0];
             best_rout = i;
          }
      }  

      if(b > threshold){
          b = 900;
 	  for(i = 0; i < 12 && neighbors[i] != 99; i++){
             if(cmmbcr_mets[i][1] < b && cmmbcr_mets[i][0] > threshold){
                 b = cmmbcr_mets[i][1];
                 best_rout = i;
             }
          }
      }
      router_battery = cmmbcr_mets[best_rout][0];
      local_hops = cmmbcr_mets[best_rout][1];	
      set_router(neighbors[best_rout]);
  }
}



void update_router(const rout_msg_t * msg){
  /* Here is the basic routing algorithm. You shall design a better one below. */
 if(ROUTERTYPE == 0 || !has_router()) {
    /* I do not have a router ==> use the first neighbor that is closer to the sink as router */
    bool good_candidate = distance_to_sink(msg->from) < distance_to_sink(node_id);
    if( !has_router() && good_candidate ){
      router_battery = msg->bat;
      router_distance = distance_to_sink(msg->from);
      router_battery  = msg->bat;
      local_hops = msg->hops + 1;
      set_router(msg->from);
    }
  } else if(ROUTERTYPE == 3) {
  /* Here is where you take a better decision.
    * Set BASICROUTER to 0 and your algorithm runs instead.
    * You might have to change in other places as well.
    * It's nice if you can switch back and forth by setting
    * BASICROUTER, but it's not a requirement.
    */

    double msg_metric = 1.0;
    int8_t i;
    if(msg->hops > 0){
       double bt = ((double)msg->bat)/(BATTERYSTART*msg->hops);
       for(i=0;i<msg->hops;i++){
         msg_metric *= bt;
       }
      // msg_metric = ((double)msg->bat)/(BATTERYSTART*msg->hops);
    }else{
       msg_metric = 1.0;
    }

   // update_metric(msg->from, msg_metric);
    if(router_addr.u16[0] == msg->from){
        // current router is same, we just update battery
        router_battery = msg->bat;
        router_metric = msg_metric;

    } else {
        double rng = ((double) randommmm(100))/100;
        //if(!has_router() ||( (distance_to_sink(msg->from) < distance_to_sink(node_id)) && router_battery < msg->bat)){
        if(((distance_to_sink(msg->from) < distance_to_sink(node_id)) && rng <= msg_metric)){
            local_hops = msg->hops+1;
            router_battery = msg->bat;
            router_metric = msg_metric;
            set_router(msg->from);
	    LOG_INFO("HUG LIFE. msg->bat: %u, router_battery: %u, current_battery: %u", msg->bat, router_battery, current_battery);
        }
    }


  } else if(ROUTERTYPE == 1){
    //LOG_INFO("LET BRIGITTE SAY FUCK");
    if(router_addr.u16[0] == msg->from){
        // current router is same, we just update battery
        router_battery = msg->bat;
        local_hops = msg->hops+1;
        router_metric = msg->batt;
    } else {
	/*double r_bat = (double) router_battery;
	double m_bat = (double) msg->bat;
  
        if(msg->hops > 0){
          m_bat = m_bat/msg->hops;
        }

        if(local_hops - 1 > 0){
	  r_bat = r_bat/(local_hops - 1);
        }*/

        if(distance_to_sink(msg->from) < distance_to_sink(node_id) && msg->hops+1 <= local_hops){ 
            local_hops = msg->hops+1;
            router_battery = msg->bat;
            //router_metric = msg_metric;
            set_router(msg->from);
            LOG_INFO("HUG LIFE. msg->bat: %u, router_battery: %u, current_battery: %u", msg->bat, router_battery, current_battery);
        }
  }
  } else if(ROUTERTYPE == 2){
      int32_t msg_distance = distance_to_sink(msg->from);
      if(msg_distance < router_distance && msg_distance < local_distance){
          router_battery = msg->bat;
          router_distance = msg_distance;
          set_router(msg->from);
      }
      if(msg_distance < local_distance && msg->bat > router_battery){
          router_battery = msg->bat;
          router_distance = msg_distance;
          set_router(msg->from);
      }
  }else if(ROUTERTYPE == 4){
    double msg_metric = 1.0;
  //  int8_t i;
    if(msg->hops > 0){
       //double bt = ((double)msg->bat)/BATTERYSTART;
      // for(i=0;i<distance_to_sink(msg->from);i++){
      //   msg_metric *= bt;
      // }
      msg_metric = msg->hops;//distance_to_sink(msg->from); //* (((double) msg->bat)/BATTERYSTART);
    }else{
       msg_metric = 1.0;
    }

    if(distance_to_sink(msg->from) < distance_to_sink(node_id)){
      update_metric(msg->from, msg_metric);
    }

  }else if(ROUTERTYPE == 5){
    if(router_addr.u16[0] == msg->from){
        // current router is same, we just update battery
        router_battery = msg->bat;
    } else {
        if((can_reach_sink(node_id) && msg->from==SINKNODE) ||
           (!can_reach_sink(node_id) && 
            distance_to_sink(msg->from) < distance_to_sink(node_id) && 
            msg->bat < router_battery)){
             router_battery = msg->bat;
             set_router(msg->from);
            
        }
  }

  }else if(ROUTERTYPE == 6){
     LOG_INFO("UPDATE DAT METRIC");
     if(distance_to_sink(msg->from) < distance_to_sink(node_id)){
        update_metrics_cmmbcr(msg->from, msg->bat, msg->hops);
     }
  }
}

bool send_message(const rout_msg_t * msg, const linkaddr_t *nexthop_addr){
  /* *
  * Send message:
  * 1. point to the message buffer
  * 2. set the size of the message (cannot exceed ~100 bytes)
  * 3. call the send function and pass the address of the nexthop (router)
  * */
  if(nexthop_addr != NULL){
    LOG_INFO("Sending %s msg %u-%u via ", GET_MSG_TYPE_STR(msg->type), msg->from, msg->seq);
    LOG_INFO_LLADDR(nexthop_addr);
    LOG_INFO_("\n");
    nullnet_buf = (uint8_t *)msg;
    nullnet_len = sizeof(rout_msg_t);
    NETSTACK_NETWORK.output(nexthop_addr);
    return TRUE;
  } else {
    LOG_INFO("WARNING: Skipping msg %u-%u as I do not have a router\n", msg->from, msg->seq);
    return FALSE;
  }
}
/* Decide nexthop address, then send message. */
bool route_message(const rout_msg_t * msg){
  const linkaddr_t *nexthop_addr = NULL; /* pointer to constant data */
  /* *
   * send ANNOUNCEMENT as broadcast
   * and CONTENT as unicast via the router
   * */
  if( msg->type == TYPE_ANNOUNCEMENT ){
    nexthop_addr = (linkaddr_t *)&linkaddr_null; /* broadcast address -- all zeros */
  } else if( msg->type == TYPE_CONTENT ){
    //if( !has_router() ){
      if( can_reach_sink(node_id) ){ /* I can reach the sink directly */
        nexthop_addr = &sink_addr;
        LOG_INFO_("WARNING: No router is set. Trying to reach to sink directly\n");
     // }
    } else {
      nexthop_addr = &router_addr;
    }
  }
  return send_message(msg, nexthop_addr);
}

/*---------------------------------------------------------------------------*/
/* Communication functions */

bool send_content(){
  static rout_msg_t message = {0};
  static uint32_t sequence = 0;
  message.from = node_id; /* The ID of the node */
  message.type = TYPE_CONTENT;
  message.content = 1;
  message.seq = sequence++;

  if(AGGREGATION && is_aggregator()){
    message.content = agg_sum + 1;
    agg_sum = 0;
  }


  if(ROUTERTYPE == 4 && !is_sink()){
     select_router();
  }else if(ROUTERTYPE == 6 && !is_sink()){
     select_router_cmmbcr();
  }

  return route_message(&message);
}

bool send_announcement(){
  static rout_msg_t message = {0};
  static uint32_t sequence = 0;
  message.from = node_id; /* The ID of the node */
  message.type = TYPE_ANNOUNCEMENT;
  if(ROUTERTYPE == 1){
     static uint16_t rng;
     rng  = randommmm(500);
     LOG_INFO("RNG: %u", rng);
     message.hops = local_hops * (uint16_t) (rng * (((double)current_battery)/BATTERYSTART));
     //LOG_INFO("BATTERYSTART/current_battery: %f\n", ((double)current_battery)/BATTERYSTART);
     LOG_INFO("RNG: %u RNG * battery%: %u\n", rng, (uint16_t) (rng * (((double)current_battery)/BATTERYSTART)));
     LOG_INFO("RNG < 500: %u, RNG > 500: %u, RNG > 1000: %u", rng < 500, rng > 500, rng > 1000);
  }else{
     message.hops = local_hops;
  }

  if(AGGREGATION){
    if(!is_aggregator()){
        if(randommmm(100) < 100*(((double)current_battery)/BATTERYSTART)){
          message.hops = 0;
          set_aggregator(1);
    	}
    }else{
       if(randommmm(100) > 100 *(((double)current_battery)/BATTERYSTART)){
	  set_aggregator(0);
       }
    }
  }
  if(ROUTERTYPE == 5){
    
    static uint16_t resid;
    resid = (uint16_t) 100000 * (1.0/ (100.0 * ((double)current_battery/BATTERYSTART)));
    LOG_INFO("BATTERY IS %u, METRIC IS %u, batteryper is %u\n", current_battery, resid,
                           (uint16_t) (100.0 * ((double)current_battery/BATTERYSTART)));
 
    if(resid > router_battery){
       message.bat = resid;
    }else{
       message.bat = router_battery;
    }
    
  }else if(ROUTERTYPE == 6){
    static uint16_t resid;
    resid = (uint16_t) (100.0 * ((double)current_battery/BATTERYSTART));
    LOG_INFO("BATTERY IS %u, METRIC IS %u, batteryper is %u\n", current_battery, resid,
                           (uint16_t) (100.0 * ((double)current_battery/BATTERYSTART)));
    if(can_reach_sink(node_id) || resid < router_battery){
       message.bat = resid;
    }else{
       message.bat = router_battery;
    }

  }else{
     message.bat = current_battery;  
  }
  message.content = 0; /* could be used for the routing metric */
  message.seq = sequence++;
  LOG_INFO("Hops : %u, battery : %u\n",local_hops,current_battery);
  return route_message(&message);
}

bool communication_round(){
  if( has_router() || is_sink() ){
    roundcounter++;
    if( roundcounter % ROUNDS == ROUND_ANNOUNCEMENT ){
      return send_announcement();
    } else if( roundcounter % ROUNDS == ROUND_CONTENT ){
      if( !is_sink() ) {
        /* all nodes send data to reach the sink */
        return send_content();
      }
    }
  }
  return FALSE;
}

void receive_content(const rout_msg_t *incoming_message) {
  if( is_sink() ){
    sink_collect_data( incoming_message );
  }else if(AGGREGATION && is_aggregator()){
    agg_sum += incoming_message->content;
  } else{
    route_message( incoming_message );
  }
}

void receive_announcement(const rout_msg_t *incoming_message) {
  if( !is_sink() ){
    update_router( incoming_message );
  }
}

/* input_callback gets invoked when a message is received successfuly.
  data: pointer to payload,
  len: length of the payload in bytes,
  src: immediate sender address,
  dest: immediate receiver address  */
void input_callback(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest)
{
  if(len == sizeof(rout_msg_t)) {
    const rout_msg_t *incoming_message = (rout_msg_t *) data;

    LOG_DBG("RX %u %u %u %u\n", incoming_message->seq, incoming_message->from, incoming_message->content, incoming_message->type );
    LOG_INFO("Received %s msg from %u-%u via ", GET_MSG_TYPE_STR(incoming_message->type), incoming_message->from, incoming_message->seq);
    LOG_INFO_LLADDR(src);
    LOG_INFO_("\n");

    if( incoming_message->type == TYPE_ANNOUNCEMENT ){
      receive_announcement((rout_msg_t *)incoming_message);
    } else if( incoming_message->type == TYPE_CONTENT ){
      receive_content((rout_msg_t *)incoming_message);
    } else {
      LOG_INFO_("UNKNOWN message received\n");
    }
  } else {
    LOG_INFO_("STRANGE message received\n");
  }
}
/*---------------------------------------------------------------------------*/
/* STARTUP */
void start_node() {
  battery = BATTERYSTART;
  current_battery = battery;
#if MAC_CONF_WITH_TSCH
  tsch_set_coordinator(linkaddr_cmp(&sink_addr, &linkaddr_node_addr));
#endif /* MAC_CONF_WITH_TSCH */
  if(can_reach_sink(node_id)){
      local_hops=1;
  }
  if (is_sink()){
      local_hops=0;
  }
  local_distance=distance_to_sink(node_id);

  uint8_t i;
  for(i = 0; i < 12; i++){
     neighbors[i] = 99;
  }

  /* Initialize NullNet input callback */
  nullnet_set_input_callback(input_callback);
  /* Start the timer */
  etimer_set(&periodic_timer, SEND_INTERVAL);
}

void stop_node() {
  battery = 0;
  etimer_stop(&periodic_timer);
  NETSTACK_MAC.off();
}
/*---------------------------------------------------------------------------*/
/* BATTERY */
/* True if battery ran out */
bool battery_isempty() {
  return (USEBATTERY && battery == 0);
}

/* Stop node when battery has run out */
bool battery_check() {
  if (battery_isempty()) {
    LOG_INFO_("Battery: Node ran out of battery\n");
    if(node_id == SINKNODE){
      /* Do not change this message as it is used by Cooja to halt the simulation early */
      LOG_WARN_("Sink: Battery: DEAD\n");
    }
    if(ROUTERTYPE==0){
        LOG_INFO("BASIC ROUTER\n");
    }
    stop_node();
  }
  return battery_isempty();
}

/* Uses the stated level of battery.
 * Returns whether it was enough or not.
 * Shuts the node down if battery is empty
 */
bool battery_use(uint64_t use) {
  bool dostuff;
  dostuff = (use < battery);
  if(dostuff) {
    LOG_INFO("BatteryUse: Consumed %llu of battery %llu\n", use, battery);
    current_battery = battery-use;
    LOG_INFO("Current battery level : %u\n", current_battery);
  } else {
    battery = 0;
    battery_check();
  }
  return !USEBATTERY || dostuff;
}

static unsigned long to_milliseconds(uint64_t time){
  return (unsigned long)(time / (ENERGEST_SECOND/1000));
}

/* Prints used energy so far, updates estimate and checks battery: if empty then it stops the node */
static bool refresh_and_show_energy_estimate(){
  uint64_t usage;

  /* Update all energest times. */
  energest_flush();

  LOG_DBG("Energest: Radio LISTEN %4lu us, TRANSMIT %4lu us\n",
      to_milliseconds(energest_type_time(ENERGEST_TYPE_LISTEN)),
      to_milliseconds(energest_type_time(ENERGEST_TYPE_TRANSMIT)));

  usage = energest_type_time(ENERGEST_TYPE_LISTEN) * RADIO_RX_CURRENT + energest_type_time(ENERGEST_TYPE_TRANSMIT) * RADIO_TX_CURRENT;

  return battery_use(usage);
}
/*---------------------------------------------------------------------------*/
PROCESS(collect_example_process, "Collect example");
AUTOSTART_PROCESSES(&collect_example_process);
/*---------------------------------------------------------------------------*/
/* Main process function */
PROCESS_THREAD(collect_example_process, ev, data)
{
  /* *
   * use static or global variables ONLY inside the process;
   * otherwise, local variables values are lost after PROCESS_WAIT_* statements
   * (think of the PROCESS_WAIT_* statement as a return statement that exits the function, but has the ability of resuming execution from the exit point)
   * */

  static bool continue_work = 0;

  PROCESS_BEGIN();

  LOG_INFO_("Starting the application process.\n");

  start_node();

  /* keep running while we have energy */
  while( !battery_check() ) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    communication_round();
    continue_work = refresh_and_show_energy_estimate();
    if(continue_work){
      etimer_reset(&periodic_timer);
    } else {
      break;
    }
  }

  LOG_INFO_("END!\n");
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
