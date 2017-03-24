
(cl:in-package :asdf)

(defsystem "click_to_geolocate-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IntList" :depends-on ("_package_IntList"))
    (:file "_package_IntList" :depends-on ("_package"))
    (:file "FloatList" :depends-on ("_package_FloatList"))
    (:file "_package_FloatList" :depends-on ("_package"))
  ))