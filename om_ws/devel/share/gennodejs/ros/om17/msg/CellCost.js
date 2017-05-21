// Auto-generated. Do not edit!

// (in-package om17.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class CellCost {
  constructor() {
    this.x = 0;
    this.y = 0;
    this.cost = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type CellCost
    // Serialize message field [x]
    bufferInfo = _serializer.uint16(obj.x, bufferInfo);
    // Serialize message field [y]
    bufferInfo = _serializer.uint16(obj.y, bufferInfo);
    // Serialize message field [cost]
    bufferInfo = _serializer.float32(obj.cost, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type CellCost
    let tmp;
    let len;
    let data = new CellCost();
    // Deserialize message field [x]
    tmp = _deserializer.uint16(buffer);
    data.x = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [y]
    tmp = _deserializer.uint16(buffer);
    data.y = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [cost]
    tmp = _deserializer.float32(buffer);
    data.cost = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'om17/CellCost';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e3fe3b7498573a946fcc553657ef0ef';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16 x
    uint16 y
    float32 cost
    
    `;
  }

};

module.exports = CellCost;
