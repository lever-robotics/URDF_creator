// import React, { useEffect, useRef, useState } from 'react';
// import './MenuModal.css';

// const MenuModal = ({ isOpen, onClose }) => {
//   const [isVisible, setIsVisible] = useState(false);
//   const overlayRef = useRef(null);
//   const contentRef = useRef(null);

//   useEffect(() => {
//     if (isVisible) {
//       const overlay = overlayRef.current;
//       const content = contentRef.current;

//       if (overlay && content) {
//         requestAnimationFrame(() => {
//           overlay.classList.add('open');
//           content.classList.add('open');
//         });
//       }
//     } else {
//       const overlay = overlayRef.current;
//       const content = contentRef.current;

//       if (overlay && content) {
//         overlay.classList.remove('open');
//         content.classList.remove('open');
//       }
//     }
//   }, [isOpen]);

//   const handleClose = () => {
//     const overlay = overlayRef.current;
//     const content = contentRef.current;

//     if (overlay && content) {
//       overlay.classList.remove('open');
//       content.classList.remove('open');
//       setTimeout(() => setIsVisible(false), 300); // Match the duration of the CSS transition
//       onClose();
//     }
//   };

//   if (!isVisible) return null;

//   return (
//     <div className="modal-overlay" onClick={handleClose} ref={overlayRef}>
//       <div className="modal-content" onClick={(e) => e.stopPropagation()} ref={contentRef}>
//         <button className="close-button" onClick={handleClose}>
//           &times;
//         </button>
//         <div className="modal-body">
//           <h2>Lightbox Modal</h2>
//           <p>This is the content of the lightbox modal.</p>
//         </div>
//       </div>
//     </div>
//   );
// };

// export default MenuModal;




