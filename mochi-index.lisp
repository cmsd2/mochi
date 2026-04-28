(in-package :cl-info)
(let (
(deffn-defvr-pairs '(
; CONTENT: (<INDEX TOPIC> . (<FILENAME> <BYTE OFFSET> <LENGTH IN CHARACTERS> <NODE NAME>))
("mod_get" . ("mochi.info" 3698 98 "Function mod_get m key"))
("mod_load" . ("mochi.info" 1522 768 "Function mod_load path"))
("mod_print" . ("mochi.info" 2523 64 "Function mod_print m"))
("mod_state_space" . ("mochi.info" 2767 46 "Function mod_state_space m op_point"))
("mod_state_space <1>" . ("mochi.info" 2995 521 "Function mod_state_space m op_point override_params"))
))
(section-pairs '(
; CONTENT: (<NODE NAME> . (<FILENAME> <BYTE OFFSET> <LENGTH IN CHARACTERS>))
("Caveat: ‘mod_load’ calls ‘kill(...)’ on every model symbol" . ("mochi.info" 4214 1036))
("Definitions for mochi" . ("mochi.info" 1177 238))
("Example" . ("mochi.info" 3943 158))
("Introduction to mochi" . ("mochi.info" 534 519))
)))
(load-info-hashtables (maxima::maxima-load-pathname-directory) deffn-defvr-pairs section-pairs))
