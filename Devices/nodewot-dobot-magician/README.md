# Base node-wot

This is a project for making the proccess of creating a WoT enabled device easier by giving base functions and code and putting fill in comments.
You should start from this project by copying it and adapting it according to your specific application

## Adapting to your needs

### Changing package.json file
  * Open package.json file in the root of the folder
  * Change name of the package
  * Change description
  * Add needed dependencies: Here you should add other bindings of node-wot if needed

### Changing base.ts
  * Open src/base.ts
  * Fill in empty quotation marks in produce function
  * Fill in addProperties, addActions and addEvents if needed

### If CoAP or MQTT is needed 
  * Add dependency @node-wot/binding-xxx (e.g. binding-http, binding-mqtt) in your package.json
  * Uncomment related lines in index.js

### Follow installation steps 

## Installation

- Get the latest node.js: 
```bash
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get install -y nodejs
```
You will need to change 10.x to a newer version if needed
- To install dependencies: `npm install`
- To build (transcompiling Typescript to javascript): `npm run build`
- To run the code: `npm run start` 
