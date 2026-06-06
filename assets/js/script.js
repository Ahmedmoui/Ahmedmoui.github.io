document.addEventListener('DOMContentLoaded', function() {
    const menuToggle = document.querySelector('.menu-toggle');
    const mobileNav = document.querySelector('.mobile-nav');

    menuToggle.addEventListener('click', function() {
        menuToggle.classList.toggle('active');
        mobileNav.classList.toggle('active');
    });

    // Close menu when clicking outside
    document.addEventListener('click', function(event) {
        if (!event.target.closest('nav')) {
            menuToggle.classList.remove('active');
            mobileNav.classList.remove('active');
        }
    });

    // Sticky nav shadow on scroll
    const nav = document.querySelector('nav');
    window.addEventListener('scroll', function() {
        nav.classList.toggle('scrolled', window.scrollY > 10);
    });

    // Scroll-triggered reveal for cards and sections
    const revealTargets = document.querySelectorAll('.project-card, .skills-card, .contact-section');

    // Fallback: if IntersectionObserver isn't supported, reveal everything
    // immediately so nothing stays hidden.
    if (!('IntersectionObserver' in window)) {
        revealTargets.forEach(function(el) { el.classList.add('visible'); });
    } else {
        const revealObserver = new IntersectionObserver(function(entries) {
            entries.forEach(function(entry) {
                if (entry.isIntersecting) {
                    entry.target.classList.add('visible');
                    revealObserver.unobserve(entry.target);
                }
            });
        }, { threshold: 0, rootMargin: '0px 0px -30px 0px' });

        revealTargets.forEach(function(el) { revealObserver.observe(el); });
    }
});
