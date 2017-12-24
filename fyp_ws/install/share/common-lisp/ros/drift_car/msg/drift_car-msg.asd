
(cl:in-package :asdf)

(defsystem "drift_car-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Action" :depends-on ("_package_Action"))
    (:file "_package_Action" :depends-on ("_package"))
    (:file "IMUData" :depends-on ("_package_IMUData"))
    (:file "_package_IMUData" :depends-on ("_package"))
  ))