Overview
++++++++

提供機能
--------

- HSRBでMoveItを使用するための、launch群を提供する

How to use
++++++++++

Launchファイル
----------------

Rvizのみのシミュレータで動作確認する場合は、以下のlaunchを起動する。
(実機や、Gazeboがなくても確認できる)

.. code-block:: bash

   roslaunch hsrb_moveit_config hsrb_demo.launch

MoveItから直接、実機、Gazebo等実際のコントローラに対して指令を発行するには、以下のlaunchを起動する。

.. code-block:: bash

   roslaunch hsrb_moveit_config hsrb_demo_with_controller.launch

LICENSE
+++++++

This software is released under the BSD 3-Clause Clear License, see LICENSE.txt.
