---
layout: about
title: about
permalink: /
subtitle: Accelerating Forward Autonomous Future | CTRL, OCP, GNC on Vehicle, Robot

profile:
  align: right
  image: ZengQT_Ghibli.png
  image_circular: false # crops the image to make it circular
  more_info: >
    <p><a href="mailto:zengqt.e@gmail.com"><i class="fa-solid fa-envelope"></i> zengqt.e@gmail.com</a></p>
    <p><i class="fa-solid fa-location-dot"></i> ShangHai, China</p>
    <p><a href="https://github.com/QingtanZeng" target="_blank"><i class="fa-brands fa-github"></i> github.com/QingtanZeng</a></p>

selected_papers: false # includes a list of papers marked as "selected={true}"
social: false # includes social icons at the bottom of the page

announcements:
  enabled: false # includes a list of news items
  scrollable: false # adds a vertical scroll bar if there are more than 3 news items
  limit: 5 # leave blank to include all the news in the `_news` folder

latest_posts:
  enabled: false
  scrollable: false # adds a vertical scroll bar if there are more than 3 new posts items
  limit: 3 # leave blank to include all the blog posts

---

Hi, I’m an Engineer specializing in autonomous systems. My R&D work focuses on the intersection of 
trajectory generation, optimization and motor control, dedicated to achieving real-time, robust planning and control of autonomous systems including robots and vehicles.

Upon graduating from Jilin University with a Bachelor's degree in Vehicle Engineering in 2019, 
I went on to complete a Master of Science degree in Mechanical Engineering at Beijing Institute of Technology's 
National Engineering Research Center for Electric Vehicles in 2022. 

Subsequently, I served as a Motor Control Software Engineer responsible for 
PMSM control of NEVs' electric drive axle at United Automotive Electronic Systems (a Bosch subsidiary) 
from 2022 to 2025, where I was officially certified as a Departmental Expert.


<br><br><br>

## Experience
<div style="margin-bottom: 25px;"> <div style="display: flex; justify-content: space-between; align-items: flex-start;">
    <div style="font-weight: bold; font-size: 1.1em;">United Automotive Electronic Systems (UAES)</div>
    <div style="color: gray; font-size: 0.9em; min-width: 110px; text-align: right;">2022 – 2025</div>
  </div>
  <div style="font-style: italic; color: #555;">Motor Control Software Engineer | Departmental Expert</div>
</div>

<div style="margin-bottom: 25px;">
  <div style="display: flex; justify-content: space-between; align-items: flex-start;">
    <div style="font-weight: bold; font-size: 1.1em;">Beijing Institute of Technology (BIT)</div>
    <div style="color: gray; font-size: 0.9em; min-width: 110px; text-align: right;">2019 – 2022</div>
  </div>
  <div style="font-style: italic; color: #555;">M.Sc. in Mechanical Engineering (NERC-EV)</div>
</div>

<div style="margin-bottom: 25px;">
  <div style="display: flex; justify-content: space-between; align-items: flex-start;">
    <div style="font-weight: bold; font-size: 1.1em;">Jilin University</div>
    <div style="color: gray; font-size: 0.9em; min-width: 110px; text-align: right;">2015 – 2019</div>
  </div>
  <div style="font-style: italic; color: #555;">B.Eng. in Vehicle Engineering</div>
</div>

## Projects
<div class="projects">
  {% assign selected_projects = site.projects | where: "category", "work" | sort: "importance" %}
  <div class="container">
    <div class="row row-cols-1 row-cols-md-3">
      {% for project in selected_projects %}
        {% include projects.liquid %}
      {% endfor %}
    </div>
  </div>
</div>
