
(cl:in-package :asdf)

(defsystem "cv_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "grasp_msg" :depends-on ("_package_grasp_msg"))
    (:file "_package_grasp_msg" :depends-on ("_package"))
  ))