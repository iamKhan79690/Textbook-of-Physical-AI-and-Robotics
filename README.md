# Textbook of Physical AI and Robotics

A comprehensive AI-Native Physical Robotics Textbook built with [Docusaurus](https://docusaurus.io/).

This project includes:
- 📚 **Interactive Textbook**: Comprehensive content on Physical AI and Robotics
- 🤖 **RAG Server**: Python-based Retrieval-Augmented Generation server for AI-powered content queries
- 📖 **Docusaurus Frontend**: Modern, searchable documentation interface

## Installation

```bash
cd book-source
npm install
```

## Local Development

```bash
cd book-source
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## RAG Server Setup

```bash
cd rag-server
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

## Build

```bash
cd book-source
npm run build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Project Structure

- `/book-source` - Docusaurus-based textbook frontend
  - `/docs` - Book chapters and content
  - `/blog` - Blog posts and updates
  - `/src` - Custom React components
  - `/static` - Static assets (images, files)
- `/rag-server` - Python RAG server for AI-powered queries
- `/specs` - Project specifications and documentation

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is open source and available under the MIT License.
