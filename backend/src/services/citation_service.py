"""
Citation generation logic for source references
"""
from typing import List, Dict, Any
from ..models.retrieval import RetrievalResult
from ..schemas.response import Citation


class CitationService:
    """Service for generating and formatting citations from retrieval results"""

    @staticmethod
    def format_citations_from_retrieval_results(
        retrieval_results: List[Dict[str, Any]]
    ) -> List[Citation]:
        """
        Format retrieval results into citation objects
        """
        citations = []
        for result in retrieval_results:
            citation = Citation(
                chapter=result.get('chapter', 'Unknown'),
                section=result.get('section', 'Unknown'),
                similarity_score=result.get('similarity_score', 0.0)
            )
            citations.append(citation)
        return citations

    @staticmethod
    def format_citations_from_retrieval_entities(
        retrieval_entities: List[RetrievalResult]
    ) -> List[Dict[str, Any]]:
        """
        Format retrieval entities into citation dictionaries
        """
        citations = []
        for entity in retrieval_entities:
            # Parse the source_document field which is in format "Chapter X/Section Y"
            source_parts = entity.source_document.split('/')
            chapter = source_parts[0] if source_parts else "Unknown"
            section = source_parts[1] if len(source_parts) > 1 else "Unknown"

            citations.append({
                "chapter": chapter,
                "section": section,
                "similarity_score": entity.similarity_score
            })
        return citations

    @staticmethod
    def generate_provenance_statement(
        citations: List[Dict[str, Any]],
        query_text: str = None
    ) -> str:
        """
        Generate a provenance statement for the response
        """
        if not citations:
            return "No specific sources cited for this response."

        if len(citations) == 1:
            citation = citations[0]
            return f"Based on content from {citation['chapter']}, {citation['section']}"
        else:
            # Get unique chapters
            chapters = set()
            sections_by_chapter = {}

            for citation in citations:
                chapter = citation['chapter']
                section = citation['section']
                chapters.add(chapter)

                if chapter not in sections_by_chapter:
                    sections_by_chapter[chapter] = []
                sections_by_chapter[chapter].append(section)

            # Format the statement
            parts = []
            for chapter in sorted(chapters):
                sections = sorted(sections_by_chapter[chapter])
                if len(sections) == 1:
                    parts.append(f"{chapter} {sections[0]}")
                else:
                    parts.append(f"{chapter} ({', '.join(sections)})")

            return f"Based on content from {', '.join(parts)}"

    @staticmethod
    def sort_citations_by_relevance(citations: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Sort citations by relevance (similarity score, descending)
        """
        return sorted(citations, key=lambda x: x.get('similarity_score', 0.0), reverse=True)

    @staticmethod
    def filter_citations_by_threshold(
        citations: List[Dict[str, Any]],
        threshold: float = 0.5
    ) -> List[Dict[str, Any]]:
        """
        Filter citations to only include those above a relevance threshold
        """
        return [citation for citation in citations
                if citation.get('similarity_score', 0.0) >= threshold]

    @staticmethod
    def get_most_relevant_citation(
        citations: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Get the most relevant citation from a list
        """
        if not citations:
            return {}
        return max(citations, key=lambda x: x.get('similarity_score', 0.0))