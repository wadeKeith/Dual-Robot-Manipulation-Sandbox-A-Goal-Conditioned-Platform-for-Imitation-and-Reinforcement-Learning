// Auto-generated. Do not edit!

// (in-package move_demo.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetJointAnglesRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.elbow_joint = null;
      this.shoulder_lift_joint = null;
      this.shoulder_pan_joint = null;
      this.wrist_1_joint = null;
      this.wrist_2_joint = null;
      this.wrist_3_joint = null;
    }
    else {
      if (initObj.hasOwnProperty('elbow_joint')) {
        this.elbow_joint = initObj.elbow_joint
      }
      else {
        this.elbow_joint = 0.0;
      }
      if (initObj.hasOwnProperty('shoulder_lift_joint')) {
        this.shoulder_lift_joint = initObj.shoulder_lift_joint
      }
      else {
        this.shoulder_lift_joint = 0.0;
      }
      if (initObj.hasOwnProperty('shoulder_pan_joint')) {
        this.shoulder_pan_joint = initObj.shoulder_pan_joint
      }
      else {
        this.shoulder_pan_joint = 0.0;
      }
      if (initObj.hasOwnProperty('wrist_1_joint')) {
        this.wrist_1_joint = initObj.wrist_1_joint
      }
      else {
        this.wrist_1_joint = 0.0;
      }
      if (initObj.hasOwnProperty('wrist_2_joint')) {
        this.wrist_2_joint = initObj.wrist_2_joint
      }
      else {
        this.wrist_2_joint = 0.0;
      }
      if (initObj.hasOwnProperty('wrist_3_joint')) {
        this.wrist_3_joint = initObj.wrist_3_joint
      }
      else {
        this.wrist_3_joint = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointAnglesRequest
    // Serialize message field [elbow_joint]
    bufferOffset = _serializer.float64(obj.elbow_joint, buffer, bufferOffset);
    // Serialize message field [shoulder_lift_joint]
    bufferOffset = _serializer.float64(obj.shoulder_lift_joint, buffer, bufferOffset);
    // Serialize message field [shoulder_pan_joint]
    bufferOffset = _serializer.float64(obj.shoulder_pan_joint, buffer, bufferOffset);
    // Serialize message field [wrist_1_joint]
    bufferOffset = _serializer.float64(obj.wrist_1_joint, buffer, bufferOffset);
    // Serialize message field [wrist_2_joint]
    bufferOffset = _serializer.float64(obj.wrist_2_joint, buffer, bufferOffset);
    // Serialize message field [wrist_3_joint]
    bufferOffset = _serializer.float64(obj.wrist_3_joint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointAnglesRequest
    let len;
    let data = new SetJointAnglesRequest(null);
    // Deserialize message field [elbow_joint]
    data.elbow_joint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [shoulder_lift_joint]
    data.shoulder_lift_joint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [shoulder_pan_joint]
    data.shoulder_pan_joint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wrist_1_joint]
    data.wrist_1_joint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wrist_2_joint]
    data.wrist_2_joint = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wrist_3_joint]
    data.wrist_3_joint = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_demo/SetJointAnglesRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a04d423b1f72637454d383a9cff0573';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # SetJointAngles.srv
    float64 elbow_joint
    float64 shoulder_lift_joint
    float64 shoulder_pan_joint
    float64 wrist_1_joint
    float64 wrist_2_joint
    float64 wrist_3_joint
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointAnglesRequest(null);
    if (msg.elbow_joint !== undefined) {
      resolved.elbow_joint = msg.elbow_joint;
    }
    else {
      resolved.elbow_joint = 0.0
    }

    if (msg.shoulder_lift_joint !== undefined) {
      resolved.shoulder_lift_joint = msg.shoulder_lift_joint;
    }
    else {
      resolved.shoulder_lift_joint = 0.0
    }

    if (msg.shoulder_pan_joint !== undefined) {
      resolved.shoulder_pan_joint = msg.shoulder_pan_joint;
    }
    else {
      resolved.shoulder_pan_joint = 0.0
    }

    if (msg.wrist_1_joint !== undefined) {
      resolved.wrist_1_joint = msg.wrist_1_joint;
    }
    else {
      resolved.wrist_1_joint = 0.0
    }

    if (msg.wrist_2_joint !== undefined) {
      resolved.wrist_2_joint = msg.wrist_2_joint;
    }
    else {
      resolved.wrist_2_joint = 0.0
    }

    if (msg.wrist_3_joint !== undefined) {
      resolved.wrist_3_joint = msg.wrist_3_joint;
    }
    else {
      resolved.wrist_3_joint = 0.0
    }

    return resolved;
    }
};

class SetJointAnglesResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointAnglesResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointAnglesResponse
    let len;
    let data = new SetJointAnglesResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'move_demo/SetJointAnglesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointAnglesResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetJointAnglesRequest,
  Response: SetJointAnglesResponse,
  md5sum() { return '8b430f70f7ff32529e5429ef4104cf78'; },
  datatype() { return 'move_demo/SetJointAngles'; }
};
