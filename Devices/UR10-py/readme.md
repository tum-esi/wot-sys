
## Program Flow
&nbsp;
* replace.py converts the thing model in TM.json to thing description and wites it into convertedTD.json.

    * Thing model in TM.json can be configured using config.json 

&nbsp;

* ur10_flask.py is the main server script which reads the thing description from convertedTD.json

    * Some constant values in ur10_flask.py is read from constants.json