#!/bin/python

import csv
import requests
import sys
import datetime
import random

templateSkyMote='''
<mote>
  <breakpoints />
  <interface_config>
    org.contikios.cooja.interfaces.Position
    <x>%f</x>
    <y>%f</y>
    <z>0.0</z>
  </interface_config>
  <interface_config>
    org.contikios.cooja.mspmote.interfaces.MspClock
    <deviation>1.0</deviation>
  </interface_config>
  <interface_config>
    org.contikios.cooja.mspmote.interfaces.MspMoteID
    <id>%d</id>
  </interface_config>
  <motetype_identifier>sky1</motetype_identifier>
</mote>
'''
templateCoojaMote='''
<mote>
  <interface_config>
    org.contikios.cooja.interfaces.Position
    <x>%f</x>
    <y>%f</y>
    <z>0.0</z>
  </interface_config>
  <interface_config>
    org.contikios.cooja.contikimote.interfaces.ContikiMoteID
    <id>%d</id>
  </interface_config>
  <interface_config>
    org.contikios.cooja.contikimote.interfaces.ContikiRadio
    <bitrate>250.0</bitrate>
  </interface_config>
  <interface_config>
    org.contikios.cooja.contikimote.interfaces.ContikiEEPROM
    <eeprom>AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA==</eeprom>
  </interface_config>
  <motetype_identifier>mtype311</motetype_identifier>
</mote>
'''

#Parameters: random seed, integer
headerCoojaMote='''<?xml version="1.0" encoding="UTF-8"?>
<simconf>
  <project EXPORT="discard">[APPS_DIR]/mrm</project>
  <project EXPORT="discard">[APPS_DIR]/mspsim</project>
  <project EXPORT="discard">[APPS_DIR]/avrora</project>
  <project EXPORT="discard">[APPS_DIR]/serial_socket</project>
  <project EXPORT="discard">[APPS_DIR]/powertracker</project>
  <simulation>
    <title>Advanced distributed systems - lab 2 - grid</title>
    <randomseed>%d</randomseed>
    <motedelay_us>1000000</motedelay_us>
    <radiomedium>
      org.contikios.cooja.radiomediums.UDGM
      <transmitting_range>50.0</transmitting_range>
      <interference_range>100.0</interference_range>
      <success_ratio_tx>1.0</success_ratio_tx>
      <success_ratio_rx>1.0</success_ratio_rx>
    </radiomedium>
    <events>
      <logoutput>4000000</logoutput>
    </events>
    <motetype>
      org.contikios.cooja.contikimote.ContikiMoteType
      <identifier>mtype311</identifier>
      <description>Cooja Mote Type #1</description>
      <source>[CONTIKI_DIR]/examples/adv-ds-lab2/nullnet-unicast.c</source>
      <commands>make nullnet-unicast.cooja TARGET=cooja -j</commands>
      <moteinterface>org.contikios.cooja.interfaces.Position</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Battery</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiVib</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiMoteID</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRS232</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiBeeper</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.RimeAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiIPAddress</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiRadio</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiButton</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiPIR</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiClock</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiLED</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiCFS</moteinterface>
      <moteinterface>org.contikios.cooja.contikimote.interfaces.ContikiEEPROM</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.Mote2MoteRelations</moteinterface>
      <moteinterface>org.contikios.cooja.interfaces.MoteAttributes</moteinterface>
      <symbols>false</symbols>
    </motetype>
'''

#Parameter: script timeout, integer
footerCoojaMote = '''
  </simulation>
  <plugin>
    org.contikios.cooja.plugins.SimControl
    <width>280</width>
    <z>3</z>
    <height>160</height>
    <location_x>400</location_x>
    <location_y>0</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.Visualizer
    <plugin_config>
      <moterelations>true</moterelations>
      <skin>org.contikios.cooja.plugins.skins.IDVisualizerSkin</skin>
      <skin>org.contikios.cooja.plugins.skins.GridVisualizerSkin</skin>
      <skin>org.contikios.cooja.plugins.skins.TrafficVisualizerSkin</skin>
      <skin>org.contikios.cooja.plugins.skins.UDGMVisualizerSkin</skin>
      <viewport>2.625519771195162 0.0 0.0 2.625519771195162 -933.0374451842179 33.852254753800665</viewport>
    </plugin_config>
    <width>430</width>
    <z>1</z>
    <height>400</height>
    <location_x>1</location_x>
    <location_y>1</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.LogListener
    <plugin_config>
      <filter />
      <formatted_time />
      <coloring />
    </plugin_config>
    <width>1455</width>
    <z>4</z>
    <height>240</height>
    <location_x>400</location_x>
    <location_y>160</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.Notes
    <plugin_config>
      <notes>Enter notes here</notes>
      <decorations>true</decorations>
    </plugin_config>
    <width>1175</width>
    <z>2</z>
    <height>160</height>
    <location_x>680</location_x>
    <location_y>0</location_y>
  </plugin>
  <plugin>
    org.contikios.cooja.plugins.ScriptRunner
    <plugin_config>
      <script>/*
 * Example Contiki test script (JavaScript).
 * A Contiki test script acts on mote output, such as via printf()'s.
 * The script may operate on the following variables:
 *  Mote mote, int id, String msg
 */

TIMEOUT(%d);

while (true) {
  log.log(time + ":" + id + ":" + msg + "\\n");
  if(msg.contains("Sink: Battery: DEAD")){
    log.testOK(); /* Report test success and quit */
    //log.testFailed(); /* Report test failure and quit */
  }
  YIELD();
}</script>
      <active>false</active>
    </plugin_config>
    <width>600</width>
    <z>0</z>
    <height>604</height>
    <location_x>710</location_x>
    <location_y>30</location_y>
  </plugin>
</simconf>
'''

def generateCoojaMoteList(type='cooja'):
  global templateCoojaMote, templateSkyMote, headerCoojaMote, footerCoojaMote

  headerFileName='motes-xy-lookup-table.h'
  coojaFileName='nullnet-unicast-cooja-script-test.csc'
  coojaRandomSeed = 123456
  coojaScriptTimeoutMinutes=10
  numberOfMotes = 50
  yspace=25
  xspace=25
  rows=4

  coojaFileContents = ""
  MOTES_XY_LOOKUP_TABLE_contents = ""
  template = templateCoojaMote if type == 'cooja' else templateSkyMote
  coojaScriptTimeout = coojaScriptTimeoutMinutes*60*1000 #in milliseconds

  x=0.0
  y=0.0
  ymod=rows*yspace
  randomize=0

  nlist = range(1, numberOfMotes+1, 1)
  positions = []
  coojaFileContents = coojaFileContents + headerCoojaMote % (coojaRandomSeed)

  for n in nlist:
    x = x + randomize * random.randint(-xspace/2, xspace/2)
    y = y + randomize * random.randint(-yspace/2, yspace/2)
    positions.append((x, y))
    coojaFileContents = coojaFileContents + template % (x,y,n)
    y = (y + yspace) % ymod 
    if (y==0):
      x = x + xspace

  coojaFileContents = coojaFileContents + footerCoojaMote % (coojaScriptTimeout)

  MOTES_XY_LOOKUP_TABLE_contents = '''
/* Advanced distributed systems - lab 2 - grid 
 * number of motes = %d, x-space = %d, y-space = %d, randomized = %d 
 */
#ifndef MOTES_XY_LOOKUP_TABLE
#define MOTES_XY_LOOKUP_TABLE ((const uint16_t [][2]){ \\
''' % (numberOfMotes, xspace, yspace, randomize)

  for p in positions:
    MOTES_XY_LOOKUP_TABLE_contents = MOTES_XY_LOOKUP_TABLE_contents + "{%d,%d}, \\\n" %(int(p[0]), int(p[1]))
  
  MOTES_XY_LOOKUP_TABLE_contents = MOTES_XY_LOOKUP_TABLE_contents + '''})
#endif /* MOTES_XY_LOOKUP_TABLE */
'''

  print coojaFileContents
  open(coojaFileName , 'w').write(coojaFileContents)

  print MOTES_XY_LOOKUP_TABLE_contents
  open(headerFileName , 'w').write(MOTES_XY_LOOKUP_TABLE_contents)

if __name__ == '__main__':
  generateCoojaMoteList('cooja')



