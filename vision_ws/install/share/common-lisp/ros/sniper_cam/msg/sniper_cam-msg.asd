
(cl:in-package :asdf)

(defsystem "sniper_cam-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "interopImages" :depends-on ("_package_interopImages"))
    (:file "_package_interopImages" :depends-on ("_package"))
  ))