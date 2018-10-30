# WoT-SYS

This is a project that contains the source code, description, guides etc. of the WoT System that will be used by students of ESI.

Useful Links:
1. [Thing Description Specification](https://w3c.github.io/wot-thing-description/#thing)
2. [Tool to validate Thing Descriptions](http://plugfest.thingweb.io/playground/)
3. [Scripting API Specification](https://w3c.github.io/wot-scripting-api/)
4. [Library that implements the Scripting API](https://github.com/eclipse/thingweb.node-wot)

# installation
- follow the instructions on thingweb.node-wot
- add /usr/local/lib/node_modules to NODE_PATH (`export NODE_PATH=$NODE_PATH:/usr/local/lib/node_modules`)
- also install `onoff` (for GPIO) and `pi-camera` (for camera) for demonstration purposes
- then `npm install`
- `node index.js`, and one should see the TD on `localhost:8080`
