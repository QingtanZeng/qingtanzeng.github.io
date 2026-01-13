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

  <div class="row row-cols-1 row-cols-md-4">
    {% for project in sorted_projects %}
      {% include projects.liquid %}
    {% endfor %}
  </div>
</div>

<style>
  /* 修改项目卡片标题字体大小 */
  .projects .card-title {
    font-size: 1.0rem !important; /* 默认通常是 1.25rem 或更大，您可以调小这个数字 */
    line-height: 1.2;             /* 调整行高，防止文字挤在一起 */
    font-weight: bold;            /* 保持加粗 */
  }
</style>