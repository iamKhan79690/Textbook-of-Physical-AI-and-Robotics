declare module '*.module.css' {
    const classes: { [key: string]: string };
    export default classes;
}

declare module '*.css' {
    const content: { [key: string]: string };
    export default content;
}

// Docusaurus theme modules
declare module '@theme/Layout';
declare module '@theme/*';
declare module '@theme-original/*';
declare module '@site/*';
declare module '@docusaurus/*';
declare module '@generated/*';
