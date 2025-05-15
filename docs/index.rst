Welcome to the UiAbot documentation!
====================================

.. figure:: src/fig/uiabot_photo.png
   :width: 500
   :align: center

   UiAbot AMR platform.

Acknowledgment
--------------
Inspired by TurtleBot3, Prof. Dr. `Daniel Hagen <https://www.uia.no/en/kk/profile/danielh>`_ developed the UiAbot from 2021 to 2022 in collaboration with Mechatronics students. Funding for the necessary equipment and the development of this mobile robot platform was generously provided by the Mechatronics Section at the University of Agder.   

**Publication:** This work has been published in IEEE ICIEA 2024 <https://ieeexplore.ieee.org/document/10664856>_. To cite, please use the following BibTeX:

.. code-block:: bibtex

   @INPROCEEDINGS{Hagen2024,
     author={Hagen, D. and Mæland, M. and Skotterud, T. and Dahlseng Hermansen, M. and Haij, H. and Sauar Wad, M.},
     booktitle={2024 IEEE 19th Conference on Industrial Electronics and Applications (ICIEA)},
     title={UiAbot: A Versatile Autonomous Mobile Robot Platform for Research and Education},
     year={2024},
     pages={1-8},
     doi={10.1109/ICIEA61579.2024.10664856}}

.. toctree::
   :hidden:
   :maxdepth: 3
   :caption: Getting Started:

   src/Overview
   src/installation

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: Implementation and Testing:

   src/motion_control
   src/perception
   src/localization_mapping
   src/localization_navigation

.. toctree::
   :hidden:
   :maxdepth: 2
   :caption: More Information:

   src/troubleshooting
   src/software_documentation