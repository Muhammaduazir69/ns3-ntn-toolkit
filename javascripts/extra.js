// ns3-ntn-toolkit — lightweight interactivity (no external deps)
(function () {
  "use strict";

  // ---- Click-to-zoom lightbox for gallery media ----
  function initLightbox() {
    var imgs = document.querySelectorAll(".showcase-grid img, .ntn-hero-banner img");
    if (!imgs.length) return;
    var overlay = document.createElement("div");
    overlay.className = "ntn-lightbox";
    overlay.innerHTML = '<span class="ntn-lightbox-close" aria-label="Close">&times;</span><img alt="">';
    document.body.appendChild(overlay);
    var big = overlay.querySelector("img");
    function open(src, alt) {
      big.src = src; big.alt = alt || "";
      overlay.classList.add("open");
      document.body.style.overflow = "hidden";
    }
    function close() {
      overlay.classList.remove("open");
      document.body.style.overflow = "";
      big.src = "";
    }
    imgs.forEach(function (im) {
      im.style.cursor = "zoom-in";
      im.addEventListener("click", function () { open(im.src, im.alt); });
    });
    overlay.addEventListener("click", close);
    document.addEventListener("keydown", function (e) { if (e.key === "Escape") close(); });
  }

  // ---- Count-up animation for stat numbers ----
  function initCounters() {
    var nums = document.querySelectorAll(".ntn-stats .n[data-target]");
    if (!nums.length || !("IntersectionObserver" in window)) return;
    var io = new IntersectionObserver(function (entries) {
      entries.forEach(function (en) {
        if (!en.isIntersecting) return;
        var el = en.target, target = parseFloat(el.dataset.target),
            suffix = el.dataset.suffix || "", dur = 1100, t0 = null;
        function step(ts) {
          if (!t0) t0 = ts;
          var p = Math.min((ts - t0) / dur, 1),
              val = Math.floor((target >= 1000 ? Math.round(target * p / 10) * 10 : target * p));
          el.textContent = val.toLocaleString() + suffix;
          if (p < 1) requestAnimationFrame(step);
          else el.textContent = target.toLocaleString() + suffix;
        }
        requestAnimationFrame(step);
        io.unobserve(el);
      });
    }, { threshold: 0.4 });
    nums.forEach(function (n) { io.observe(n); });
  }

  // mkdocs-material instant navigation: re-init on each page load
  if (typeof document$ !== "undefined" && document$.subscribe) {
    document$.subscribe(function () { initLightbox(); initCounters(); });
  } else {
    document.addEventListener("DOMContentLoaded", function () { initLightbox(); initCounters(); });
  }
})();
