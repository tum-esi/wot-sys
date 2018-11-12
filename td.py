# this request python >= 3.5
from flask import Flask, jsonify, request, Response
from flask.json import JSONEncoder

class Property(object):
    def __init__(self, name, value_type = None, metadata = {}):
        self.name = name
        self.type = value_type
        self.metadata = metadata
        self.forms = []
        self.properties = []

    def add_form(self, href, contenttype = 'application/json', metadata = {}):
        form = {}
        form['href'] = href
        form['contenttype'] = contenttype
        self.forms.append({ **form, **metadata})
        return self # for a calling chain :)

    def add_property(self,prop):
        self.properties.append(prop)
        return self

    def serialize(self):
        td = {}

        if self.type:
            td['type'] = self.type
            if self.type == 'object' and len(self.properties) > 0:
                td['properties'] = self.properties
        
        return { **td, **self.metadata, 'forms': self.forms}

class Thing(object):
    def __init__(self, thing_id, name, description = "",security = None, metadata = {}):
       self.thing_id = thing_id
       self.name = name
       self.description = description
       self.metadata = metadata

       self.properties = []
       self.actions = []
       self.security = [{ 'scheme': 'nosec'}] if not security else security

    def _string_or_empty_string(self,s):
        return s if s else ''

    def add_property(self,prop):
        self.properties.append(prop)
        return self

    def add_actions(self,action):
        self.actions.append(action)
        return self

    # the function that generates thing descriptions given the properties
    def serialize(self):
        td = {}
        
        td['@context'] = 'http://www.w3.org/ns/td'
        td['id'] = self.thing_id
        td['name'] = self.name
        td['description'] = self._string_or_empty_string(self.description)
        td['properties'] = {prop.name: prop.serialize() for prop in self.properties}
        td['actions'] = {action.name: action.serialize() for action in self.actions}
        td['security'] = self.security

        # merge both the meta data and compulsory fields
        return {**td, **self.metadata}

class TDServer(object):
    
    class EndpointAction(object):
        def __init__(self,action):
            self.action = action
            self.response = Response(status = 200, headers = {})

        def __call__(self,*args):
            response = self.action()
            return response if response else self.response

    def __init__(self,things):
        self.app = Flask(__name__)
        self.things = []
        # create the main point for 'listing' the list of things 
        for thing in things:
            self.add_thing(thing)
        self._add_endpoint("/", "_summary", self.root_handler)

    def root_handler(self):
        if len(self.things) == 1:
            return jsonify(self.things[0].serialize())
        else:
            # TODO: how about multipe things?
            return {}

    def run(self,host = '0.0.0.0',port = 5000):
        self.app.run(host = host,port = port)

    # helper function for creating an endpoint
    def _add_endpoint(self,endpoint = None, endpoint_name = None, handler = None):
        print("adding endpoint {}".format(endpoint_name))
        self.app.add_url_rule(endpoint,endpoint_name, TDServer.EndpointAction(handler))
    
    # helper function for creating endpoints given a new thing added
    def add_thing(self,thing):
        self.things.append(thing)
        sub_space_to_underscore = lambda s: s.replace(' ','_')
        thing_url_name = sub_space_to_underscore(thing.name)
        # the handler for this particular thing
        def handler():
            return jsonify(thing.serialize())
        self._add_endpoint(
                '/{}'.format(thing_url_name),
                thing_url_name,
                lambda: jsonify(thing.serialize())
        )
        # endpoints for properties of each thing
        for prop in thing.properties:
            prop_url_name = sub_space_to_underscore(prop.name)
            self._add_endpoint(
                    '/{}/properties/{}'.format(thing_url_name,prop_url_name),
                    '{}:{}'.format(thing_url_name,prop_url_name),
                    lambda: jsonify(prop.serialize())
            )
        # endpoints for actions of each thing
        for action in thing.actions:
            prop_url_name = sub_space_to_underscore(prop.name)
            self._add_endpoint(
                    '/{}/actions/{}'.format(thing_url_name,prop_url_name),
                    '{}:{}'.format(thing_url_name,prop_url_name),
                    lambda: jsonify(action.serialize())
            )

if __name__ == '__main__':
   
   # lets hardcode some devices here...
   camera_thing = Thing(
                'esi:picamera',
                'piCamera',
                'a camera mounted on Raspberry Pi').add_property( 
                   Property('configuration','object')
                    .add_property({
                        'title': 'brightness',
                        'type': 'number',
                        'minimum': 0,
                        'maximum': 100
                    })
                    .add_property({
                        'title': 'size',
                        'type': 'array',
                        'properties': [
                        {
                            'title': 'width',
                            'type': 'number',
                            'minimum': 0
                        }, 
                        {
                            'title': 'height',
                            'type': 'number',
                            'minimum': 0
                        }],
                        'minItems': 2,
                        'maxItems': 2
                    })
                    .add_form('http://192.168.0.104:5000/properties/configuration')
                ).add_property(
                    Property('frame',None)
                    .add_form('http://192.168.0.104:5000/properties/frame','image/jpeg')
                )


   server = TDServer([camera_thing])
   server.run(port = 3000)

