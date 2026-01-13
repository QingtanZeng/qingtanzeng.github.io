---
layout: page
title: Software
permalink: /software/
description: Open Source Implementations provided by me.
nav: true
nav_order: 3
display_categories: [software]  # <--- 这里只展示 software 分类
horizontal: false
---

<div class="projects">
  {% assign categorized_projects = site.projects | where: "category", "software" %}
  {% assign sorted_projects = categorized_projects | sort: "importance" %}

  <div class="row row-cols-1 row-cols-md-2">
    {% for project in sorted_projects %}
      {% include projects.liquid %}
    {% endfor %}
  </div>
</div>