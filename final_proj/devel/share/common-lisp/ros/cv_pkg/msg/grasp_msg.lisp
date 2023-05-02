; Auto-generated. Do not edit!


(cl:in-package cv_pkg-msg)


;//! \htmlinclude grasp_msg.msg.html

(cl:defclass <grasp_msg> (roslisp-msg-protocol:ros-message)
  ((grasp_x_cm
    :reader grasp_x_cm
    :initarg :grasp_x_cm
    :type cl:float
    :initform 0.0)
   (grasp_y_cm
    :reader grasp_y_cm
    :initarg :grasp_y_cm
    :type cl:float
    :initform 0.0)
   (grasp_theta
    :reader grasp_theta
    :initarg :grasp_theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass grasp_msg (<grasp_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grasp_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grasp_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cv_pkg-msg:<grasp_msg> is deprecated: use cv_pkg-msg:grasp_msg instead.")))

(cl:ensure-generic-function 'grasp_x_cm-val :lambda-list '(m))
(cl:defmethod grasp_x_cm-val ((m <grasp_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cv_pkg-msg:grasp_x_cm-val is deprecated.  Use cv_pkg-msg:grasp_x_cm instead.")
  (grasp_x_cm m))

(cl:ensure-generic-function 'grasp_y_cm-val :lambda-list '(m))
(cl:defmethod grasp_y_cm-val ((m <grasp_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cv_pkg-msg:grasp_y_cm-val is deprecated.  Use cv_pkg-msg:grasp_y_cm instead.")
  (grasp_y_cm m))

(cl:ensure-generic-function 'grasp_theta-val :lambda-list '(m))
(cl:defmethod grasp_theta-val ((m <grasp_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cv_pkg-msg:grasp_theta-val is deprecated.  Use cv_pkg-msg:grasp_theta instead.")
  (grasp_theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grasp_msg>) ostream)
  "Serializes a message object of type '<grasp_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'grasp_x_cm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'grasp_y_cm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'grasp_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grasp_msg>) istream)
  "Deserializes a message object of type '<grasp_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasp_x_cm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasp_y_cm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grasp_theta) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grasp_msg>)))
  "Returns string type for a message object of type '<grasp_msg>"
  "cv_pkg/grasp_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grasp_msg)))
  "Returns string type for a message object of type 'grasp_msg"
  "cv_pkg/grasp_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grasp_msg>)))
  "Returns md5sum for a message object of type '<grasp_msg>"
  "4d3c4f7d62e5e5364236dabf83a36004")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grasp_msg)))
  "Returns md5sum for a message object of type 'grasp_msg"
  "4d3c4f7d62e5e5364236dabf83a36004")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grasp_msg>)))
  "Returns full string definition for message of type '<grasp_msg>"
  (cl:format cl:nil "float32 grasp_x_cm~%float32 grasp_y_cm~%float32 grasp_theta~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grasp_msg)))
  "Returns full string definition for message of type 'grasp_msg"
  (cl:format cl:nil "float32 grasp_x_cm~%float32 grasp_y_cm~%float32 grasp_theta~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grasp_msg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grasp_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'grasp_msg
    (cl:cons ':grasp_x_cm (grasp_x_cm msg))
    (cl:cons ':grasp_y_cm (grasp_y_cm msg))
    (cl:cons ':grasp_theta (grasp_theta msg))
))
