# this request python >= 3.5
from flask import Flask, jsonify, request, Response
from flask.json import JSONEncoder
from enum import Enum,unique
from typing import Type, Any, List
import json
@unique
class DataType(str,Enum):
    object = 'object',
    array = 'array',
    number = 'number',
    string = 'string',
    boolean = 'boolean',
    integer = 'integer',
    none = 'none'
    

class DataSchema(object):

    def __init__(self,
            title: str, 
            description: str = '', 
            type: Type[DataType] = DataType.none,
            unit: str = None,
            enum: list = [],
            readOnly: bool = False,
            writeOnly: bool = False
    ): # this is also an emoji..
        self.title = title
        self.description = description
        self.type = type
        self.unit = unit
        self.enum = enum
        self.readOnly = readOnly
        self.writeOnly = writeOnly

        self.properties = []
    def add_property(self, property: 'DataSchema') -> 'DataSchema':
        if self.type not in [ DataType.object, DataType.array]:
            raise TypeError("If the type of data is not 'object' or 'array', it cannot have its own properties")
        else:
            self.properties.append(property)
        return self

    def serialize(self,isTopLevel: bool = True):
        td = {}
        if isTopLevel:
            td["title"] = self.title
        td["description"] = self.description
        if self.type != DataType.none:
            td["type"] = self.type

        if self.unit:
            td["unit"] = self.unit
        if len(self.enum) > 0:
            td["enum"] = self.enum

        td["readOnly"] = self.readOnly
        td["writeOnly"] = self.writeOnly

        if self.type in [DataType.object, DataType.array]:
            td['properties'] = {prop.title: prop.serialize(False) for prop in self.properties}

        return td

class BooleanSchema(DataSchema):
    def __init__(self,
            title: str, 
            description: str,
            readOnly: bool = False,
            writeOnly: bool = False
    ):
        DataSchema.__init__(self,
                title, 
                description,
                DataType.boolean,
                readOnly = readOnly,
                writeOnly = writeOnly)

class ObjectSchema(DataSchema):
    def __init__(self,
            title: str, 
            description: str = '',
            required: List[str] = [],
            type: Type[DataType] = DataType.none,
            *args
    ):
        DataSchema.__init__(self,
            title, 
            description, 
            DataType.object,
            *args
        )
        self.required = required
    def serialize(self,*args):
        td = super().serialize(*args)
        td['required'] = self.required
        return td
    def add_property(self, prop: DataSchema,required: bool = False):
        if required:
            self.required.append(prop.title)
        return super().add_property(prop)

class NumberSchema(DataSchema):
    def __init__(self,
            title: str, 
            description: str = "",
            minimum: float = None, 
            maximum: float = None,
            *args):
        DataSchema.__init__(self, title, description, type = DataType.number,*args)
        self.minimum = minimum
        self.maximum = maximum

    def set_minimum(self,num: float):
        self.minimum = num
        return self

    def set_maximum(self,num: float):
        self.maximum = num
        return self

    def serialize(self,*args):
        td = super().serialize(*args)
        if self.minimum is not None:
            td['minimum'] = self.minimum
        if self.maximum is not None:
            td['maximum'] = self.maximum
        return td

class ArraySchema(DataSchema):
    def __init__(self, title: str, *args):
        DataSchema.__init__(self,title,type = DataType.array, *args)
        self.min_items = None
        self.max_items = None

    def set_min_items(self,num: int):
        self.min_items = num
        return self
    
    def set_max_items(self,num: int):
        self.max_items = num
        return self

    def serialize(self, *args):
        td = super().serialize(*args)
        if self.min_items:
            td['minItems'] = self.min_items
        if self.max_items:
            td['maxItems'] = self.max_items
        return td

class Property(DataSchema):
    def __init__(self, 
            title: str, 
            description: str = '',
            type: DataType = DataType.none,
            observable: bool = False,
            *args
    ):
        DataSchema.__init__(self,
                title,
                description,
                type,
                observable,
                *args)
        self.forms = []

    def add_form(self, href, contenttype = 'application/json', metadata = {}):
        form = {}
        form['href'] = href
        form['contenttype'] = contenttype
        self.forms.append({ **form, **metadata})
        return self # for a calling chain :)

    def add_property(self,prop: DataSchema):
        self.properties.append(prop)
        return self

    def serialize(self,*args):
        td = super().serialize(*args) 
        return { **td, 'forms': self.forms}

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

    def add_property(self,prop: Property):
        self.properties.append(prop)
        return self

    def add_actions(self,action):
        self.actions.append(action)
        return self

    # the function that generates thing descriptions given the properties
    def serialize(self):
        td = {}
        
        td['id'] = self.thing_id
        td['name'] = self.name
        td['description'] = self._string_or_empty_string(self.description)
        td['properties'] = {prop.title: prop.serialize(False) for prop in self.properties}
        td['actions'] = {action.title: action.serialize(False) for action in self.actions}
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
            prop_url_name = sub_space_to_underscore(prop.title)
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

   
def generate_camera_thing(dev_addr):
   return Thing(
            'esi:picamera',
            'piCamera',
            'a camera mounted on Raspberry Pi').add_property( 
               Property(
                   'configuration',
                   'configuration of the camera',
                   DataType.object
                )
                .add_property(
                    NumberSchema(
                        "brightness",
                        "brightness of the camera", 
                    )
                    .set_minimum(0)
                    .set_maximum(100)
                )
                .add_property(
                    ObjectSchema(
                        "size",
                        "size (width, height) of the frame",
                    )
                    .add_property(
                        NumberSchema(
                            "width",
                            "width of the camera", 
                        ).set_minimum(0),
                        required = True
                    ) 
                    .add_property(
                        NumberSchema(
                            "height",
                            "height of the camera"
                        )
                        .set_minimum(0),
                        required = True
                    )
                )
                .add_form('http://{}:5000/properties/configuration'.format(dev_addr))
            ).add_property(
                Property('frame',"frame of the current camera")
                .add_form('http://{}:5000/properties/frame'.format(dev_addr),'image/jpeg')
            )


