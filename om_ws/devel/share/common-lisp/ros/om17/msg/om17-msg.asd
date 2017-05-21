
(cl:in-package :asdf)

(defsystem "om17-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CellCost" :depends-on ("_package_CellCost"))
    (:file "_package_CellCost" :depends-on ("_package"))
    (:file "CellCost" :depends-on ("_package_CellCost"))
    (:file "_package_CellCost" :depends-on ("_package"))
  ))