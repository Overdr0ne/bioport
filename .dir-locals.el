((nil .
      ((multi-compile-alist . (
                               ("\\.*" . (("build" . "cd /home/sam/src/bioport && west build .")
                                          ("flash" . "cd /home/sam/src/bioport && west flash")
                                          ("pristine" . "cd /home/sam/src/bioport && west build -p && cp ./build/compile_commands.json"))))))))
;; (nil . ((eval .(setq multi-compile-alist '(
;;                             ("\\.*" . (("build" . "cd build && ninja flash")
;;                                        ("make" . "cd build && ninja flash"))))))))

;; ((string/starts-with buffer-file-name "/home/sam/src/zephyr/bioport")) . ("build" . "ninja flash")))
