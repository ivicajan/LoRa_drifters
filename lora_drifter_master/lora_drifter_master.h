#pragma once

#include "src/lora_drifter_libs/lora_drifter_data_types.h"

// String literal for the main web page. Alter the processor function to display 
// different html elements on the web page. e.g. %MASTER%.
static const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>UWA LoRa Drifters</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="refresh" content="1" >
    <link rel="icon" href="data:,">
    <style>
      html {
        font-family: Arial; display: inline-block; text-align: center;
      }
      h2 {
        font-size: 3.0rem;
      }
      p {
        font-size: 3.0rem;
      }
      table {
        width: 100%%;
      }
      table, th, td {
        border: 1px solid black;
      }
      body {
        max-width: 90%%; margin:0px auto; padding-bottom: 25px;
      }
    </style>
  </head>
  <body>
    <h2>LoRa Drifters</h2>
    <h4>Master Node</h4>
    <h5>Battery %BATTERYPERCENT% %%</h5>
    <table>
      <tr>
        <td><b>GPS Time</b></td>
        <td><b>Lon</b></td>
        <td><b>Lat</b></td>
        <td><b>GPS Age [ms]</b></td>
        <td><b>Download Data </b></td>
        <td><b>Last File Write GPS Time</b></td>
        <td><b>Erase Data (NO WARNING)</b></td>
      </tr>
      %MASTER%
    </table>
    %SERVANTS%
    %DIAGNOSTICS%
    </table>
    %STATUSFLAGS%
    </table>
    %MESSAGELOG%
    </table>
  </body>
</html>
)rawliteral";
