
(cl:in-package :asdf)

(defsystem "cylRecognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "objects" :depends-on ("_package_objects"))
    (:file "_package_objects" :depends-on ("_package"))
  ))